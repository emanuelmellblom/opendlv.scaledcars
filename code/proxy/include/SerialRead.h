#ifndef SerialRead_H_
#define SerialRead_H_


#include <iostream>
#include <opendavinci/odcore/io/StringListener.h>
#include <opendavinci/odcore/base/Thread.h>
#include <string>


	class SerialRead :public odcore::io::StringListener {
		public:
			SerialRead();
			virtual ~SerialRead();
			virtual void nextString(const std::string &s);
			std::string readstr();
			char get(int);
		private:
			std::string output;
			char buff[8];
		};


#endif /*PROXY_H_*/
