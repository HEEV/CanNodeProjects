#include <string>
#include <iostream>
#include <fstream>
#include <ostream>

using namespace std;
extern int CSV[3000] = { 1, 3, 2,1 };
int main() {




	string temp;
	//open the csv 
	//TODO verify openign correctly
	ofstream out("CSVAdjusted.h");
	ifstream in("CSVAdjusted.csv");
	int index=0;
	int firstNumber;
	int secondNumber;
	int commaPosition;
	if (out.is_open() && in.is_open()) {

		//write the .h heading
		out << "int CSV[] = { ";


		out << 0;



		while (!in.eof() && index <= 4094) {
			getline(in, temp);
			commaPosition = temp.find(',');
			firstNumber = stoi(temp.substr(0, commaPosition));
			secondNumber = stoi(temp.substr(commaPosition + 1, 100));

			if (firstNumber == index) {
				out << ", " << secondNumber;
				getline(in, temp);
			}
			else {
				out << ", " << secondNumber;
			}
			
			index++;
		}
		

	}

	//get the first value of the csv
	//make sure the csv matches the value of the for loop
	//print it to the csv







	//terminate the .h and close it
	out << "}";
	out << endl << "int size ="<<index <<";";
	in.close();
	out.close();




	return 0;






}