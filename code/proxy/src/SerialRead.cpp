 
 #include "SerialRead.h"

  

 
		using namespace std;
        /*using namespace odcore::base;
        using namespace odcore::data;
        using namespace odtools::recorder;*/
		using namespace odcore;
		using namespace odcore::wrapper;

		 
		 SerialRead::SerialRead():odometerCounter(0){
				try{
					odoMem=std::shared_ptr<SharedMemory> (SharedMemoryFactory::createSharedMemory("odoMem", 2));
					//this->=static_cast<char*>(sharedMem->getSharedMemory());
					
					if (odoMem->isValid()) {
						cerr<<"valid memory \n";

					}else{
						cerr<<"MEMORY for odometer is invalid \n";
					}
				}catch(string &exception) {
					cerr << "Failed to establish shared memory for odometer " << exception << endl;
				}
		}
		
		SerialRead::~SerialRead(){
						//cerr << "go zo " << endl;
		}
		
		
		void SerialRead::incrementOdometer(char input){
			odometerCounter+=(input&31);
			
			char *p = static_cast<char*>(odoMem->getSharedMemory());
			{
				odcore::base::Lock l(odoMem);
				if(p[0]==0){
					
					if(odometerCounter >= 255){
						p[0]=(char)255;
						odometerCounter-=255;
					}else{
						p[0]=(char)odometerCounter;
						odometerCounter=0;
					}
					
				}
				
				if(p[1] > 0){
					p[0]=0;
					p[1]=0;
					odometerCounter=0;
				}
			}
			
			
		}


		void SerialRead::nextString(const string &s) {
					//cerr << "go go ";
					//*s="";
					//output=s;
					//cout << "Received " << s.length() << " bytes containing '" << s << "'" << endl;

					for(unsigned int i = 0; i < s.length(); i++){
						if((((s.at(i) & 224) >> 5) & 0x07)!=ODOMETER)
							buff[((s.at(i) & 224) >> 5) & 0x07] = s.at(i);
						else
							incrementOdometer(s.at(i));
							
					}
		}

		char SerialRead::get(int i){
			return buff[i];
		}

		std::string SerialRead::readstr() {
					//cerr << "go go ";
					//*s="";
				//	std::string temp=this->output;
					//this->output="";
					return "";
		}

