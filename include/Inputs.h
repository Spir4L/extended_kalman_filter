#include "CSVReader.h"

struct u {
    double timestamp;
    double tau;
    double z_dd;
    double theta_d;
};


/*
Measurements Class
*/
class Inputs {
    
    protected :

    std::string fileName;
    std::vector<std::vector<std::string> > dataList;
    int idx = 1;

    public:
        
        Inputs(std::string filename) : fileName(filename)
        { 
            // Creating an object of CSVReader
            CSVReader reader(filename);
            // getting data
            dataList = reader.getData();
        };
    
        u getInputData();

};

u Inputs::getInputData(){

    u new_u;

    if (idx >= dataList.size()){
        new_u.timestamp = -1;
    }
    else{
        std::vector<std::string> vec = dataList[idx];
        new_u.timestamp = std::stod(vec[0]);
        new_u.tau = std::stod(vec[1]);
        new_u.z_dd = std::stod(vec[2]);
        new_u.theta_d = std::stod(vec[3]);    
    
        idx++;
    }
    return new_u;
    
}

