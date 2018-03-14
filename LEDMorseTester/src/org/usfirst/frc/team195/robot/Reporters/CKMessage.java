package org.usfirst.frc.team195.robot.Reporters;

public class CKMessage {
	public MessageLevel messageLevel;
	public String message;

	public CKMessage(String message, MessageLevel messageLevel) {
		this.message = message;
		this.messageLevel = messageLevel;
	}
}