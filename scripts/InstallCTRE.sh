#!/bin/bash
# InstallCTRE.sh
# A script to automate the installation process of the CTRE Libraries into the wpilib folder
# Questions/bugs -> Robert Hilton (robert.a.hilton.jr@gmail.com)
# 3/4/18

DEFAULT_WPILIB_PATH="$HOME/wpilib"
CTRE_TEST_FILE="/README.txt"
CTRE_TEST_STRING="CTRE"
WPILIB_TEST_FILE="/wpilib.properties"
WPILIB_TEST_STRING="team-number"

DIR_PATH_TO_ME="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

printf '%s\n' ""
printf '%s\n' "This script will install the CTRE user libraries into your wpilib user folder."
printf '%s\n' "This operation will OVERWRITE the current CTRE libraries that are installed."
printf '%s\n' "If you do not wish to do this, please quit now."
printf '%s\n' ""

#Function checkCommandSucceeded
#Argument 1 is the result of the last command ($?)
#Argument 2 is the name of the files being copied
checkCommandSucceeded () {
	if [ $1 != 0 ]; then                  
	   echo "Installing $2 Unsuccessful! Quitting now!"
	   exit 1
	fi
}

#Ensure the user enters a valid directory for the CTRE root
while [[ -z "$CTREBase" ]]
do
	#Get the user input
    read -p "Enter CTRE folder [$DIR_PATH_TO_ME]: " CTREBase
    CTREBase="${CTREBase/#\~/$HOME}"
    
    #Use the default value if the user input is not valid
    if [ ! -d "$CTREBase" ]; then
    	CTREBase=$DIR_PATH_TO_ME
    
		if [ ! -d "$CTREBase" ]; then
			printf '%s\n' "Default value $CTREBase is not a valid directory!"
			CTREBase=""
			continue
		else
	   		printf '%s\n' "Using default value: $DIR_PATH_TO_ME"
		fi
	fi
    
    #Check that the test file exists and contains the test contents
    if [ ! -f "$CTREBase/$CTRE_TEST_FILE" ] || ! grep -q "$CTRE_TEST_STRING" "$CTREBase/$CTRE_TEST_FILE" ; then
  		printf '%s\n' "Failed to identify $CTREBase as CTRE install files! (Please check to ensure this is correct.)"
  		CTREBase=""
	fi
done 
cd "$CTREBase"
checkCommandSucceeded $? "CTRE Update"


#Ensure the user enters a valid directory for the wpilib root
while [[ -z "$wpilib" ]]
do
	#Get the user input
	read -p "Enter wpilib root [$DEFAULT_WPILIB_PATH]: " wpilib
	wpilib="${wpilib/#\~/$HOME}"

	#Use the default value if the user input is not valid
	if [ ! -d "$wpilib" ]; then
   		wpilib="$DEFAULT_WPILIB_PATH"
   		
   		if [ ! -d "$wpilib" ]; then
   			printf '%s\n' "Default value $wpilib is not a valid directory!"
   			wpilib=""
   			continue
   		else
	   		printf '%s\n' "Using default value: $wpilib"
	   	fi
   	fi
   	
   	#Check that the test file exists and contains the test contents
   	if [ ! -f "$wpilib/$WPILIB_TEST_FILE" ] || ! grep -q "$WPILIB_TEST_STRING" "$wpilib/$WPILIB_TEST_FILE" ; then
  		printf '%s\n' "Failed to identify $wpilib as the wpilib root! (Please check to ensure this is correct.)"
  		wpilib=""
	fi
done
cd "$wpilib"
checkCommandSucceeded $? "CTRE Update"


#Begin copying files, checking that each operation completed successfully
printf '%s\n' "Copying C++ Files..."
cp -rf "$CTREBase/cpp/docs/" "$wpilib/user/cpp/docs/"
checkCommandSucceeded $? "C++ Docs"
cp -rf "$CTREBase/cpp/include/" "$wpilib/user/cpp/include/"
checkCommandSucceeded $? "C++ Includes"
cp -rf "$CTREBase/cpp/lib/" "$wpilib/user/cpp/lib/"
checkCommandSucceeded $? "C++ Libraries"

printf '%s\n' "Copying Java Files..."
cp -rf "$CTREBase/java/docs/" "$wpilib/user/java/docs/"
checkCommandSucceeded $? "Java Docs"
cp -rf "$CTREBase/java/lib/" "$wpilib/user/java/lib/"
checkCommandSucceeded $? "Java Libraries"

printf '%s\n' "CTRE Files installed successfully!"
exit 0

