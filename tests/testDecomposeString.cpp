#include <iostream>
#include <string>
#include "Misc.hpp"

int
oldDecomposeString(char outWord[10],
                   char let1[10],
                   char num1[10],
                   char let2[10],
                   char num2[10],
                   char let3[10])
{
	// convert to uppercase for string matching purposes
	for (int charIdx = 0; charIdx < 10; charIdx++) {
		if (outWord[charIdx] == '\0')
			break;
		outWord[charIdx] = toupper(outWord[charIdx]);
	}

	// int wordLength = strlen(outWord);  // get length of input word (based on
	// null termination) cout << "1";
	//! find indicies of changes in number-vs-letter in characters
	unsigned int in1 =
	    strcspn(outWord, "1234567890"); // scan( OutListTmp , '1234567890' ) !
	                                    // index of first number in the string
	strncpy(let1, outWord, in1); // copy up to first number as object type
	let1[in1] = '\0';            // add null termination

	if (in1 < strlen(outWord)) // if there is a first number
	{
		// >>>>>>> the below line seems redundant - could just use in1 right???
		// <<<<<<<
		char* outWord1 =
		    strpbrk(outWord, "1234567890"); // get pointer to first number
		unsigned int il1 = strcspn(
		    outWord1,
		    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"); // in1+verify( OutListTmp(in1+1:) ,
		                                   // '1234567890' )  ! second letter
		                                   // start (assuming first character is
		                                   // a letter, i.e. in1>1)
		strncpy(num1, outWord1, il1);      // copy number
		num1[il1] = '\0';                  // add null termination

		if (il1 < strlen(outWord1)) // if there is a second letter
		{
			char* outWord2 = strpbrk(outWord1, "ABCDEFGHIJKLMNOPQRSTUVWXYZ");
			// cout << "3 il1=" << il1 << ", " ;
			unsigned int in2 =
			    strcspn(outWord2,
			            "1234567890"); // il1+scan( OutListTmp(il1+1:) ,
			                           // '1234567890' ) ! second number start
			strncpy(let2, outWord2, in2); // copy chars
			let2[in2] = '\0';             // add null termination

			if (in2 < strlen(outWord2)) // if there is a second number
			{
				char* outWord3 = strpbrk(outWord2, "1234567890");
				// cout << "4";
				unsigned int il2 = strcspn(
				    outWord3,
				    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"); // in2+verify(
				                                   // OutListTmp(in2+1:) ,
				                                   // '1234567890' )  ! third
				                                   // letter start
				strncpy(num2, outWord3, il2);      // copy number
				num2[il2] = '\0';                  // add null termination

				if (il2 < strlen(outWord3)) // if there is a third letter
				{
					char* outWord4 =
					    strpbrk(outWord3, "ABCDEFGHIJKLMNOPQRSTUVWXYZ");
					// cout << "5";
					strncpy(let3,
					        outWord4,
					        9); // copy remaining chars (should be letters)  ??
					let3[9] = '\0'; // add null termination  (hopefully takes
					                // care of case where letter D.N.E.)
				} else
					let3[0] = '\0';

			} else {
				num2[0] = '\0';
				let3[0] = '\0';
			}
		} else {
			let2[0] = '\0';
			num2[0] = '\0';
			let3[0] = '\0';
		}

	} else {
		num1[0] = '\0';
		let2[0] = '\0';
		num2[0] = '\0';
		let3[0] = '\0';

		return -1; // indicate an error because there is no number in the string
	}

	return 0;
}
/** this currently just verified the new behavior against the previous behavior
 *
 *  In cases where the input string is larger than 9 characters, we don't do any
 * check besides ensuring that they both run
 */
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
	oldDecomposeString(typeWord, let1, num1, let2, num2, let3);

	std::string str_let1, str_num1, str_let2, str_num2, str_let3;
	// divided outWord into letters and numbers
	moordyn::str::decomposeString(
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