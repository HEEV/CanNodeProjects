/*
Author: Luke Schwan
Assisted By:Samuel Ellicot
For: Supermilage Team
Purpose:Generate a lookup Table from a CSV File
*/
#include <fstream>
#include <istream>
#include <iostream>
#include <string>
using namespace std;
int main() {
//variables and streams
    string temperatureString;
    string adcString;
    const int ARRAYSIZE=3000; //RESET FOR ARRAY SIZE
    int adc;//THIS IS THE INDEX VALUE
    int commaPosition; 
    int index=0;//MATCHES THE ADC to output a value
    int temperatureInt=0;
    string temp;
    bool previousValueFound=true;
    ifstream in;
    ofstream out;

//open files change for new temp value sets
    in.open("OhmTemp.csv");
    out.open("OhmTemp.h");

//intitialize the .h
    out<<"const uint16_t tempLookupTable[]={";
    out<<endl;

//keep file open

    while(!in.eof() && index<ARRAYSIZE) {
        index++;
        if(previousValueFound) {
            getline(in,temp);// get the next value
        }
        //get the adc
        commaPosition=temp.find(',');
        //If a comma is found make sure that it does not under index the subst
        if(commaPosition>=0) {
            adc=stoi(temp.substr(0,commaPosition));
            cout<<adc<<endl;
            cout<<temp.substr(commaPosition+1,temp.length());
            temperatureInt=(100*stod(temp.substr(commaPosition+1,temp.length())));
        }
        cout<<"Your index is :" <<index<<endl;
        if(index==adc) {
            out<<temperatureInt;
            previousValueFound=true;
        }
        else {
            previousValueFound=false;
            out<<temperatureInt;//CHANGE TO ZERO TO NOT BACK INDEX

        }

        out<<",";

        if(index%5==1) {
            out<<endl;
        }



        cout<<adc;
        cout <<endl;


    }

    in.close();
//end the out file
    out<<"};";
    out.close();
    return 0;
}

