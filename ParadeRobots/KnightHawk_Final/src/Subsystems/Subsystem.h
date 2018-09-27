#ifndef SRC_SUBSYSTEMS_SUBSYSTEM_H_
#define SRC_SUBSYSTEMS_SUBSYSTEM_H_

class CustomSubsystem {
public:
	virtual void start() = 0;
	virtual ~CustomSubsystem() {};
};

#endif /* SRC_SUBSYSTEMS_SUBSYSTEM_H_ */
