#include "CSVReader.h"

struct dist {
    double x;
    double y;
    double z;
};

struct measure{
    double timestamp;
    std::vector<int> id;
    std::vector<dist> dists;
    std::vector<double> norm_dists;
}; 

/*
Measurements Class
*/
class Measurements {
    
    protected :

    std::string fileName;
    std::vector<std::vector<std::string> > dataList;
    int idx = 1;

    public:
        
        Measurements(std::string filename) : fileName(filename)
        { 
            // Creating an object of CSVReader
            CSVReader reader(filename);
            // getting data
            dataList = reader.getData();
        };
    
        measure getSensorData();

};

measure Measurements::getSensorData(){
    
    measure meas;
    std::vector<std::string> vec = dataList[idx];
    
    meas.timestamp = std::stod(vec[0]);

    for (unsigned int i = 1; i <= (vec.size()-1)/3; i++ ){
        
        vec[3*i-2].erase(std::remove(vec[3*i-2].begin(),vec[3*i-2].end(),' '),vec[3*i-2].end());
        if(!vec[3*i-2].empty()){

            meas.id.push_back(i-1);
            dist d;
            d.x = std::stod(vec[3*i-2]);
            d.y = std::stod(vec[3*i-1]);
            d.z = std::stod(vec[3*i]);
            double norm_d = sqrt(d.x*d.x + d.y*d.y +d.z*d.z);
            meas.dists.push_back(d);  
            meas.norm_dists.push_back(norm_d);  
        }

    }
    
    idx++;
    return meas;
    
}
