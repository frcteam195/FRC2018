package org.usfirst.frc.team195.robot.Utilities;

import java.util.Objects;

public class DiagnosticMessage {
	private String message;

	public DiagnosticMessage(String message) {
		this.message = message;
	}

	public String getMessage() {
		return message;
	}

	public static final DiagnosticMessage NO_MSG = new DiagnosticMessage("");

	@Override
	public String toString() {
		return message;
	}

	@Override
	public boolean equals(Object o) {
		if (this == o) return true;
		if (o == null || getClass() != o.getClass()) return false;
		DiagnosticMessage diagnosticMessage = (DiagnosticMessage) o;
		return Objects.equals(message, diagnosticMessage.message);
	}

	@Override
	public int hashCode() {
		return Objects.hash(message);
	}
}
