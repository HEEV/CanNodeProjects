#include <string>
#include <iostream>
#include <fstream>
#include <ostream>

using namespace std;
int main() {

	string temp;
    int index=0;

	//open the csv 
	//TODO verify openign correctly
	ofstream out("CSVAdjusted.h");
	ifstream in("CSVAdjusted.csv");
	if (out.is_open() && in.is_open()) {
        int firstNumber = 0;
        string secondNumber;
        int commaPosition;

		//write the .h heading
		out << "int CSV[] = { " << endl;
		while (!in.eof() ) {
            //get the next line
			getline(in, temp);

            //find the x value
			commaPosition = temp.find(',');
            //check for end
            if(commaPosition == -1) {
                break;
            }
			firstNumber = stoi(temp.substr(0, commaPosition));

            // check if some values need filled in
            if(firstNumber > index){
                //output the value found
                //if the x value is not equal to the index use the prevous
                //y value to fill in the index until we reach x
                while(index < firstNumber){
                    out  << secondNumber << ", ";
                    if(index % 10 == 0){
                        out << endl;
                    }
                    index++;
                }
            }

            //output the y value for the x value found
            //secondNumber = stoi(temp.substr(commaPosition + 1, 100));
            secondNumber = temp.substr(commaPosition + 1, temp.length()); 
            out  << secondNumber << ", ";
            //end line
            if(index % 10 == 0){
                out << endl;
            }

			index++;
		}
		

	}

	//terminate the .h and close it
	out << "}";
	out << endl << "int CSV_size ="<<index <<";";
	in.close();
	out.close();




	return 0;






}
