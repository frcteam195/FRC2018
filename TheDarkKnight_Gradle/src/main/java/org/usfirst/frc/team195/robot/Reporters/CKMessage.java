package org.usfirst.frc.team195.robot.Reporters;

import java.util.Objects;

public class CKMessage {
	public MessageLevel messageLevel;

	private String message;

	public CKMessage(String message, MessageLevel messageLevel) {
		this.message = message;
		this.messageLevel = messageLevel;
	}

	@Override
	public boolean equals(Object o) {
		if (this == o) return true;
		if (o == null || getClass() != o.getClass()) return false;
		CKMessage ckMessage = (CKMessage) o;
		return messageLevel == ckMessage.messageLevel &&
				Objects.equals(message, ckMessage.message);
	}

	@Override
	public int hashCode() {
		return Objects.hash(messageLevel, message);
	}

	@Override
	public String toString() {
		String retVal;

		switch (messageLevel) {
			case DEFCON1:
				retVal = "DEFCON1: " + message;
				break;
			case ERROR:
				retVal = "ERROR: " + message;
				break;
			case WARNING:
				retVal = "WARNING: " + message;
				break;
			case INFO:
			default:
				retVal = message;
				break;
		}

		retVal += "\n\r";
		return retVal;
	}

	public String toString(boolean appendReportingLevel) {
		return message;
	}
}