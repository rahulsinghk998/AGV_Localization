#ifndef _ENCODER_H
#define _ENCODER_H

#include <ros/ros.h>
#include <encoder/serial_lnx.h>
#include <encoder/devices.h>


namespace encoder {
	
	class EncoderData {
		
		public:
		int leftCount;
		int rightCount;
		
	};
	
	class Encoder {
		
		private:
		Tserial *serialConnection;
		
		public:
		Encoder(char *port, int baudRate);
		EncoderData fetchEncoderData();
	};
}

#endif

