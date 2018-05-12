package com.team195.utils;

import java.util.concurrent.locks.ReentrantLock;

public class VisionData {
	public static volatile double deviation = 0;
	public static volatile double distance = 0;
	public static volatile boolean targetFound = false;
	public static final ReentrantLock rl = new ReentrantLock();

	public static String printMe() {
		return deviation + " " + distance + " " + targetFound;
	}
}
