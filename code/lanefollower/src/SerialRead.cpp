 
 #include "SerialRead.h"

  

 
		using namespace std;
        /*using namespace odcore::base;
        using namespace odcore::data;
        using namespace odtools::recorder;
        using namespace odcore;
		using namespace odcore::wrapper;*/

		 
		 SerialRead::SerialRead(){
						//cerr << "go zo " << endl;
		}
		
		SerialRead::~SerialRead(){
						//cerr << "go zo " << endl;
		}


		void SerialRead::nextString(const string &s) {
					//cerr << "go go ";
					//*s="";
					 cout << "Received " << s.length() << " bytes containing '" << s << "'" << endl;
		}

	
