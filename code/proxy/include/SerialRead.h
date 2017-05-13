#ifndef SerialRead_H_
#define SerialRead_H_

#define ODOMETER 6

#include <iostream>
#include <opendavinci/odcore/io/StringListener.h>
#include <opendavinci/odcore/base/Thread.h>
#include <opendavinci/odcore/base/Lock.h>
#include <opendavinci/odcore/wrapper/SharedMemory.h>
#include <opendavinci/odcore/wrapper/SharedMemoryFactory.h>
#include <string>

using namespace odcore;
using namespace odcore::wrapper;

	class SerialRead :public odcore::io::StringListener {
		public:
			SerialRead();
			virtual ~SerialRead();
			virtual void nextString(const std::string &s);
			std::string readstr();
			char get(int);
		private:
			//std::string output;
			char buff[8];
			unsigned long odometerCounter;
			std::shared_ptr<SharedMemory> odoMem;
			void incrementOdometer(char);
		};



#endif /*PROXY_H_*/
