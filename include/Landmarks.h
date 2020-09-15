#include "CSVReader.h"
#include "../lib/eigen/Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;


struct pos {
    double x;
    double y;
    double z;
};

/*
LandMarks Class
*/
class Landmarks {
    
    protected :

    std::string fileName;
    
    public:
        
        std::vector<int> id;
        std::vector<pos> position;
        
        Landmarks(std::string filename) : fileName(filename)
        { 
            // Creating an object of CSVReader
            CSVReader reader(filename);
            // getting data
            std::vector<std::vector<std::string> > dataList = reader.getData();

            for(std::vector<std::string> vec : dataList)
            {
                if (vec[0] == "Unique ID")
                    continue;
                int id_elem = std::stoi(vec[0]);     
                this->id.push_back(id_elem);
                pos pos_elem;
                pos_elem.x = std::stod(vec[1]); 
                pos_elem.y = std::stod(vec[2]);
                pos_elem.z = std::stod(vec[3]);
                this->position.push_back(pos_elem); 
            }
        };
        Eigen::MatrixXd getMap(std::vector<int> &ids);

};


MatrixXd Landmarks::getMap(std::vector<int> &ids){


    MatrixXd result(ids.size(),3);

    for (unsigned int i = 0; i < ids.size(); i++){

        for (unsigned int j = 0; j < id.size(); j++){

            if (ids[i] == id[j]){

                result(i,0) = position[j].x;
                result(i,1) = position[j].y;
                result(i,2) = position[j].z; 
                break;
            }
            
        }    

    }
    return result;

};

