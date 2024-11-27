#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <string>


int main(){

    // file pointer
    std::fstream fout;

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::stringstream ss;

    ss << std::put_time(&tm, "%d%m%Y_%H%M%S");

    std::string date_and_time_ = ss.str();

    fout.open(date_and_time_+"_latencies.csv", std::ios::out | std::ios::app);

    std::cout << "Enter the details of 5 students:"
         << " roll name maths phy chem bio"
         << std::endl;

    int i, roll, phy, chem, math, bio;
    std::string name;

    // Read the input
    for (i = 0; i < 5; i++) {

        std::cin >> roll
            >> name
            >> math
            >> phy
            >> chem
            >> bio;

        // Insert the data to file
        fout << roll << ", "
             << name << ", "
             << math << ", "
             << phy << ", "
             << chem << ", "
             << bio
             << "\n";
    }
}