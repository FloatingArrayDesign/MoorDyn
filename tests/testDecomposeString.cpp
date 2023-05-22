#include <iostream>
#include <string>
#include "Config.h"
#include "MoorDyn2.h"
#include "MoorDyn2.hpp"
#include "MoorDynAPI.h"
#include "Misc.hpp"
bool
do_test(const std::string& source)
{
	std::cout << "Doing test with string " << std::quoted(source) << std::endl;
	char let1[10], num1[10], let2[10], num2[10], let3[10];
	char typeWord[10];
	strncpy(typeWord, source.c_str(), 9);
	typeWord[9] = '\0';
	bool inputs_equal = true;
	if (strcmp(typeWord, source.c_str()) != 0) {
		std::cout << "The input words are not equal.\n";
		std::cout << "typeWord = " << std::quoted(typeWord) << std::endl;
		std::cout << "source = " << std::quoted(source) << std::endl;
		inputs_equal = false;
	}
	// divided outWord into letters and numbers
	moordyn::str::decomposeString(typeWord, let1, num1, let2, num2, let3);

	std::string str_let1, str_num1, str_let2, str_num2, str_let3;
	// divided outWord into letters and numbers
	moordyn::str::newDecomposeString(
	    source, str_let1, str_num1, str_let2, str_num2, str_let3);

	bool cmp_let1 = strcmp(let1, str_let1.c_str()) == 0;
	bool cmp_num1 = strcmp(num1, str_num1.c_str()) == 0;
	bool cmp_let2 = strcmp(let2, str_let2.c_str()) == 0;
	bool cmp_num2 = strcmp(num2, str_num2.c_str()) == 0;
	bool cmp_let3 = strcmp(let3, str_let3.c_str()) == 0;
	if (!cmp_let1 || !cmp_num1 || !cmp_let2 || !cmp_num2 || !cmp_let3) {
		if (inputs_equal) {
			return false;
		}
		std::cout << "The output words are NOT equal.\n";
		std::cout << "let1     = " << std::quoted(let1) << std::endl;
		std::cout << "str_let1 = " << std::quoted(str_let1) << std::endl;
		std::cout << "num1     = " << std::quoted(num1) << std::endl;
		std::cout << "str_num1 = " << std::quoted(str_num1) << std::endl;
		std::cout << "let2     = " << std::quoted(let2) << std::endl;
		std::cout << "str_let2 = " << std::quoted(str_let2) << std::endl;
		std::cout << "num2     = " << std::quoted(num2) << std::endl;
		std::cout << "str_num2 = " << std::quoted(str_num2) << std::endl;
		std::cout << "let3     = " << std::quoted(let3) << std::endl;
		std::cout << "str_let3 = " << std::quoted(str_let3) << std::endl;
	} else {
		std::cout << "The outputs are equal\n";
		std::cout << "let1 = " << std::quoted(str_let1) << std::endl;
		std::cout << "num1 = " << std::quoted(str_num1) << std::endl;
		std::cout << "let2 = " << std::quoted(str_let2) << std::endl;
		std::cout << "num2 = " << std::quoted(str_num2) << std::endl;
		std::cout << "let3 = " << std::quoted(str_let3) << std::endl;
	}
	return true;
}
int
testDecomposeString()
{
	// this one won't actually ever fail because the inputs will be different
	// when Body1Pinned gets truncated to 9 chars
	if (!do_test("Body1Pinned")) {
		return 1;
	}
	if (!do_test("Body1Pin")) {
		return 2;
	}
	if (!do_test("sp 123 !a")) {
		return 3;
	}
	if (!do_test("123abc456")) {
		return 4;
	}
	if (!do_test("a1b2c3d4")) {
		return 5;
	}
	return 0;
}

int
main()
{
	return testDecomposeString();
}