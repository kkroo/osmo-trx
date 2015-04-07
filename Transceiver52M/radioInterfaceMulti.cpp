/*
 * Multi-carrier radio interface
 *
 * Copyright (C) 2015 Ettus Research LLC 
 *
 * Author: Tom Tsou <tom@tsou.cc>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * See the COPYING file in the main directory for details.
 */

#include <radioInterface.h>
#include <Logger.h>

#include "Resampler.h"

extern "C" {
#include "convert.h"
}

/* Resampling parameters for 270.833 kHz / 400 kHz */
#define RESAMP_64M_INRATE			65
#define RESAMP_64M_OUTRATE			96

/* Resampling parameters for 2.8 MHz / 2.5 MHz */
#define RESAMP_100M_INRATE			28
#define RESAMP_100M_OUTRATE			25

/* Universal resampling parameters */
#define NUMCHUNKS				24
#define MCHANS					7

static size_t resamp_inrate = 0;
static size_t inchunk = 0;
static size_t resamp_outrate = 0;
static size_t outchunk = 0;

RadioInterfaceMulti::RadioInterfaceMulti(RadioDevice *wRadio,
					   size_t sps, size_t chans)
	: RadioInterface(wRadio, sps, chans),
	  outerSendBuffer(NULL), outerRecvBuffer(NULL),
	  outerOuterSendBuffer(NULL), outerOuterRecvBuffer(NULL),
	  dnsampler0(NULL), upsampler0(NULL), channelizer(NULL),
	  dnsampler1(NULL), upsampler1(NULL), synthesis(NULL),
	  history1(NULL)
{
}

RadioInterfaceMulti::~RadioInterfaceMulti()
{
	close();
}

void RadioInterfaceMulti::close()
{
	delete outerSendBuffer;
	delete outerRecvBuffer;
	delete outerOuterSendBuffer;
	delete outerOuterRecvBuffer;
	delete dnsampler0;
	delete upsampler0;
	delete dnsampler1;
	delete upsampler1;
	delete channelizer;
	delete synthesis;
	delete history1;

	outerSendBuffer = NULL;
	outerRecvBuffer = NULL;
	outerOuterSendBuffer = NULL;
	outerOuterRecvBuffer = NULL;
	dnsampler0 = NULL;
	upsampler0 = NULL;
	dnsampler1 = NULL;
	upsampler1 = NULL;
	channelizer = NULL;
	synthesis = NULL;
	history1 = NULL;

	mReceiveFIFO.resize(0);
	powerScaling.resize(0);
	history0.resize(0);
	active.resize(0);

	RadioInterface::close();
}

static int getLogicalChan(size_t pchan, size_t chans)
{
	switch (chans) {
	case 1:
		if (pchan == 0)
			return 0;
		else
			return -1;
		break;
	case 3:
		if (pchan == 0)
			return 1;
		if (pchan == 1)
			return 0;
		if (pchan == 6)
			return 2;
		else
			return -1;
		break;
	case 5:
		if (pchan == 0)
			return 2;
		if (pchan == 1)
			return 1;
		if (pchan == 2)
			return 0;
		if (pchan == 5)
			return 4;
		if (pchan == 6)
			return 3;
		else
			return -1;
		break;
	default:
		break;
	};

	return -1;
}

static int getFreqShift(size_t chans)
{
	switch (chans) {
	case 1:
		return 0;
	case 3:
		return 1;
	case 5:
		return 2;
	default:
		break;
	};

	return -1;
}

/* Initialize I/O specific objects */
bool RadioInterfaceMulti::init(int type)
{
	float cutoff = 1.0f;

	if (mChans > 5) {
		LOG(ALERT) << "Unsupported channel configuration " << mChans;
		return false;
	}

	close();

	sendBuffer.resize(mChans);
	recvBuffer.resize(mChans);
	convertSendBuffer.resize(1);
	convertRecvBuffer.resize(1);

	mReceiveFIFO.resize(mChans);
	powerScaling.resize(mChans);
	history0.resize(mChans);
	active.resize(MCHANS);

	resamp_inrate = RESAMP_64M_INRATE;
	resamp_outrate = RESAMP_64M_OUTRATE;

	inchunk = resamp_inrate * 4;
	outchunk = resamp_outrate * 4;

	if (inchunk  * NUMCHUNKS < 157 * mSPSTx * 2) {
		LOG(ALERT) << "Invalid inner chunk size " << inchunk;
		return false;
	}

	dnsampler0 = new Resampler(resamp_inrate, resamp_outrate);
	if (!dnsampler0->init(1.0)) {
		LOG(ALERT) << "Rx resampler failed to initialize";
		return false;
	}

	upsampler0 = new Resampler(resamp_outrate, resamp_inrate * mSPSTx);
	if (!upsampler0->init(cutoff)) {
		LOG(ALERT) << "Tx resampler failed to initialize";
		return false;
	}

	dnsampler1 = new Resampler(RESAMP_100M_INRATE, RESAMP_100M_OUTRATE);
	if (!dnsampler1->init(1.0)) {
		LOG(ALERT) << "Rx resampler failed to initialize";
		return false;
	}

	upsampler1 = new Resampler(RESAMP_100M_OUTRATE, RESAMP_100M_INRATE);
	if (!upsampler1->init(1.0)) {
		LOG(ALERT) << "Tx resampler failed to initialize";
		return false;
	}

	channelizer = new Channelizer(MCHANS, outchunk);
	if (!channelizer->init()) {
		LOG(ALERT) << "Rx channelizer failed to initialize";
		return false;
	}

	synthesis = new Synthesis(MCHANS, outchunk);
	if (!synthesis->init()) {
		LOG(ALERT) << "Tx synthesis filter failed to initialize";
		return false;
	}

	/*
	 * Allocate high and low rate buffers. The high rate receive
	 * buffer and low rate transmit vectors feed into the resampler
	 * and requires headroom equivalent to the filter length. Low
	 * rate buffers are allocated in the main radio interface code.
	 */
	for (size_t i = 0; i < mChans; i++) {
		sendBuffer[i] = new RadioBuffer(NUMCHUNKS, inchunk * mSPSTx,
					        upsampler0->len(), true);
		recvBuffer[i] = new RadioBuffer(NUMCHUNKS, inchunk,
		                                0, false);
		history0[i] = new signalVector(dnsampler0->len());

		active[i] = false;
		synthesis->resetBuffer(i);
	}

	history1 = new signalVector(dnsampler1->len());

	outerSendBuffer = new signalVector(synthesis->outputLen());
	outerRecvBuffer = new signalVector(channelizer->inputLen());
	outerOuterSendBuffer = new signalVector(2400);
	outerOuterRecvBuffer = new signalVector(2400);

	convertSendBuffer[0] = new short[2 * 2400];
	convertRecvBuffer[0] = new short[2 * 2400];

	/* Configure channels */
	switch (mChans) {
	case 1:
		active[0] = true;
		break;
	case 3:
		active[0] = true;
		active[1] = true;
		active[6] = true;
		break;
	case 5:
		active[0] = true;
		active[1] = true;
		active[2] = true;
		active[5] = true;
		active[6] = true;
		break;
	default:
		LOG(ALERT) << "Unsupported channel combination";
		return false;
	}

	return true;
}

/* Receive a timestamped chunk from the device */
void RadioInterfaceMulti::pullBuffer()
{
	bool local_underrun;
	size_t num;
	float *buf;

	if (recvBuffer[0]->getFreeSegments() <= 0)
		return;

	/* Outer buffer access size is fixed */
	num = mRadio->readSamples(convertRecvBuffer,
				  outerOuterRecvBuffer->size(),
				  &overrun,
				  readTimestamp,
				  &local_underrun);
	if (num != outerOuterRecvBuffer->size()) {
		LOG(ALERT) << "Receive error " << num << ", " << channelizer->inputLen();
		return;
	}

	convert_short_float((float *) outerOuterRecvBuffer->begin(),
			    convertRecvBuffer[0],
			    outerOuterRecvBuffer->size() * 2);

	underrun |= local_underrun;
	readTimestamp += num;

	dnsampler1->rotate((float *) outerOuterRecvBuffer->begin(),
			   outerOuterRecvBuffer->size(),
			   (float *) outerRecvBuffer->begin(),
			   outerRecvBuffer->size());

	channelizer->rotate((float *) outerRecvBuffer->begin(),
			    outerRecvBuffer->size());

	for (size_t i = 0; i < MCHANS; i++) {
		if (!active[i])
			continue;

		int lchan = getLogicalChan(i, mChans);
		if (lchan < 0) {
			LOG(ALERT) << "Bad logical channel " << lchan;
			continue;
		}

		/* Update history */
		buf = channelizer->outputBuffer(i);
		size_t cLen = channelizer->outputLen();
		size_t hLen = dnsampler0->len();
		size_t hSize = 2 * hLen * sizeof(float);

		memcpy(&buf[2 * -hLen], history0[lchan]->begin(), hSize);
		memcpy(history0[lchan]->begin(), &buf[2 * (cLen - hLen)], hSize);

		/* Write to the end of the inner receive buffer */
		if (!dnsampler0->rotate(channelizer->outputBuffer(i),
					channelizer->outputLen(),
					recvBuffer[lchan]->getWriteSegment(),
					recvBuffer[lchan]->getSegmentLen())) {
			LOG(ALERT) << "Sample rate upsampling error";
		}
	}
}

/* Send a timestamped chunk to the device */
bool RadioInterfaceMulti::pushBuffer()
{
	if (sendBuffer[0]->getAvailSegments() <= 0)
		return false;

	for (size_t i = 0; i < MCHANS; i++) {
		if (!active[i]) {
			synthesis->resetBuffer(i);
			continue;
		}

		int lchan = getLogicalChan(i, mChans);

		if (!upsampler0->rotate(sendBuffer[lchan]->getReadSegment(),
				        sendBuffer[lchan]->getSegmentLen(),
				        synthesis->inputBuffer(i),
				        synthesis->inputLen())) {
			LOG(ALERT) << "Sample rate downsampling error";
		}
	}

	synthesis->rotate((float *) outerSendBuffer->begin(),
			  outerSendBuffer->size());

	upsampler1->rotate((float *) outerSendBuffer->begin(),
			   outerSendBuffer->size(),
			   (float *) outerOuterSendBuffer->begin(),
			   outerOuterSendBuffer->size());

	convert_float_short(convertSendBuffer[0],
			    (float *) outerOuterSendBuffer->begin(),
			    1.0 / (float) mChans,
			    outerOuterSendBuffer->size() * 2);

	size_t num = mRadio->writeSamples(convertSendBuffer,
					  outerOuterSendBuffer->size(),
					  &underrun,
					  writeTimestamp);
	if (num != outerOuterSendBuffer->size()) {
		LOG(ALERT) << "Transmit error " << num;
	}

	writeTimestamp += num;

	return true;
}

/* Frequency comparison limit */
#define FREQ_DELTA_LIMIT		10.0

static bool fltcmp(double a, double b)
{
	return fabs(a - b) < FREQ_DELTA_LIMIT ? true : false;
}

bool RadioInterfaceMulti::tuneTx(double freq, size_t chan)
{
  if (chan >= mChans)
    return false;

  double shift = (double) getFreqShift(mChans);

  if (!chan)
    return mRadio->setTxFreq(freq + shift * GSM_CHAN_SPACING);

  double center = mRadio->getTxFreq();
  if (!fltcmp(freq, center + (double) (chan - shift) * GSM_CHAN_SPACING))
    return false;

  return true;
}

bool RadioInterfaceMulti::tuneRx(double freq, size_t chan)
{
  if (chan >= mChans)
    return false;

  double shift = (double) getFreqShift(mChans);

  if (!chan)
    return mRadio->setRxFreq(freq + shift * GSM_CHAN_SPACING);

  double center = mRadio->getRxFreq();
  if (!fltcmp(freq, center + (double) (chan - shift) * GSM_CHAN_SPACING))
    return false;

  return true;
}

double RadioInterfaceMulti::setRxGain(double db, size_t chan)
{
  if (!chan)
    return mRadio->setRxGain(db);
  else
    return mRadio->getRxGain();
}
