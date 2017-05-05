#ifndef SerialRead_H_
#define SerialRead_H_


#include <iostream>
#include <opendavinci/odcore/io/StringListener.h>
#include <opendavinci/odcore/base/Thread.h>



	class SerialRead :public odcore::io::StringListener {
		public:
			SerialRead();
			virtual ~SerialRead();
			virtual void nextString(const std::string &s);
		  
		};


#endif /*PROXY_H_*/
