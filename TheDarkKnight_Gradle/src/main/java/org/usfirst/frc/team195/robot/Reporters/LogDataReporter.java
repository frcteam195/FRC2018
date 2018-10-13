package org.usfirst.frc.team195.robot.Reporters;

import com.illposed.osc.OSCMessage;
import com.illposed.osc.OSCPortOut;
import edu.wpi.first.wpilibj.DriverStation;
import org.usfirst.frc.team195.robot.Utilities.Constants;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.InetAddress;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class LogDataReporter {
	private static int portNumber = Constants.LOG_OSC_REPORTER_PORT;

	private static InetAddress IPAddress;
	private static OSCPortOut oscPortOut;

	public static void reportOSCData(String logData) {
		if (IPAddress != null) {
			if (oscPortOut == null) {
				try {
					oscPortOut = new OSCPortOut(IPAddress, portNumber);
				} catch (Exception e) {
					return;
				}
			}
			try {
				oscPortOut.send(createSendData(logData));
			} catch (IOException e) {
//				e.printStackTrace();
			}
		} else {
			try {
				IPAddress = InetAddress.getByName(Constants.DASHBOARD_IP);
			} catch (Exception e) {
//				e.printStackTrace();
			}
		}
	}

	private static OSCMessage createSendData(String logData) {
		ArrayList<Object> args = new ArrayList<>();
		args.add("Enabled:" + DriverStation.getInstance().isEnabled() + ";");

		String[] sArr = logData.split(";");

		for (String s: sArr)
			if (!s.isEmpty())
				args.add(s + ";");

		try {
			String[] procCmd = {
					"/bin/sh",
					"-c",
					"grep 'cpu ' /proc/stat | awk '{usage=($2+$4)*100/($2+$4+$5)} END {print usage \"%\"}'"
			};
			Process cpuProc = Runtime.getRuntime().exec(procCmd);

			cpuProc.waitFor(10, TimeUnit.MILLISECONDS);
			String s;
			StringBuilder sb = new StringBuilder();
			BufferedReader stdInput = new BufferedReader(new InputStreamReader(cpuProc.getInputStream()));
			while ((s = stdInput.readLine()) != null) {
				sb.append(s);
			}

			args.add("CPULoad_System:" + sb.toString() + ";");



			String[] memCmd = {
					"/bin/sh",
					"-c",
					"cat /proc/meminfo | awk '/Mem.*\\s*[0-9](\\d*)/{print $2}'"
			};
			Process memProc = Runtime.getRuntime().exec(memCmd);

			memProc.waitFor(10, TimeUnit.MILLISECONDS);
			s = "";
			int[] memArray = new int[3];	//Total, Free, Available mem
			stdInput = new BufferedReader(new InputStreamReader(memProc.getInputStream()));
			for (int i = 0; i < 3; i++) {
				if ((s = stdInput.readLine()) != null)
					memArray[i] = Integer.parseInt(s);
			}

			args.add("TotalMemory:" + memArray[0] + ";");
			args.add("FreeMemory:" + memArray[1] + ";");
			args.add("AvailableMemory:" + memArray[2] + ";");
		} catch (Exception e) {

		}

		return new OSCMessage("/LogData", args);
	}
}
