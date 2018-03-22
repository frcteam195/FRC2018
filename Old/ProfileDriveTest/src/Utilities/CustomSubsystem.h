#ifndef SRC_SUBSYSTEMS_CUSTOMSUBSYSTEM_H_
#define SRC_SUBSYSTEMS_CUSTOMSUBSYSTEM_H_

#include <mutex>
#include "Utilities/GlobalDefines.h"

using namespace std;

class CustomSubsystem {
public:
	virtual ~CustomSubsystem() {};
	virtual void init() = 0;
	virtual void start() = 0;
	virtual void subsystemHome() = 0;
	virtual void stop() = 0;

protected:
	mutex _subsystemMutex;
};

#endif /* SRC_SUBSYSTEMS_CUSTOMSUBSYSTEM_H_ */
