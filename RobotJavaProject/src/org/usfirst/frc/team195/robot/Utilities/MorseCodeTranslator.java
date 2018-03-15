package org.usfirst.frc.team195.robot.Utilities;

import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;

import java.util.Arrays;
import java.util.Base64;
import java.util.HashMap;
import java.util.LinkedList;

public class MorseCodeTranslator {
	private static final Character alphabet [] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', ' ', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0'};
	private static final String morseCode [] = {".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..", ".---", "-.-", ".-..", "--", "-.", "---", ".--.", "--.-", ".-.", "...", "-", "..-", "...-", ".--", "-..-", "-.--", "--..", "|", ".----", "..---", "...--", "....-", ".....", "-....", "--...", "---..", "----.", "-----"};
	private HashMap<Character, String> morseCodes = new HashMap<Character, String>();

	private static final String predefinedStrings [] = {""};
	private static final String predefinedCodes [] = {""};
	private HashMap<String, String> mappedPredefinedMorseCodes = new HashMap<String, String>();

	public MorseCodeTranslator() {
		if (alphabet.length != morseCode.length || predefinedStrings.length != predefinedCodes.length) {
			ConsoleReporter.report("Error occurred instantiating Morse Code Translator! Check your translation maps.", MessageLevel.ERROR);
			return;
		}

		for (int i = 0; i < alphabet.length; i++) {
			morseCodes.put(alphabet[i], morseCode[i]);
		}

		for (int i = 0; i < predefinedStrings.length; i++) {
			mappedPredefinedMorseCodes.put(predefinedStrings[i], predefinedCodes[i]);
		}
	}

	public LinkedList<String> getMorseCode(String input) {
		input = input.toLowerCase();
		if (mappedPredefinedMorseCodes.containsKey(input))
			return new LinkedList<String>(Arrays.asList(mappedPredefinedMorseCodes.get(input)));

		LinkedList<String> morseCodeReturn = new LinkedList<String>();
		for (Character c : input.toCharArray()) {
			morseCodeReturn.add(morseCodes.get(c));
		}
		
		return morseCodeReturn;
	}

	public LinkedList<String> getMorseCodeFromBase64(String input) {
		Base64.Decoder bD = Base64.getDecoder();
		input = new String(bD.decode(input));
		return getMorseCode(input);
	}

}
