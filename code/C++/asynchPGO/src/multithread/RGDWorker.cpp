#include <iostream>
#include <unistd.h>
#include "multithread/RGDWorker.h"


using namespace std;

namespace AsynchPGO{

	RGDWorker::RGDWorker(RGDMaster* pMaster, unsigned pId){
		id = pId;
		master = pMaster;
		mFinishRequested = false;
		mFinished = false;

		cout << "Worker " << id << " initialized. "<< endl;
	}

	void RGDWorker::run(){
		while(true){
			master->increment();

			if(mFinishRequested) break;
			
			// use usleep for microsecond
			sleep(1); 
		}

		mFinished = true;
		cout << "Worker " << id << " finished." << endl;
	}

	void RGDWorker::requestFinish(){
		mFinishRequested = true;
	}

	bool RGDWorker::isFinished(){
		return mFinished;
	}
}