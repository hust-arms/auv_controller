#ifndef COMMON_H_
#define COMMON_H_

#include <math.h>
#include <string>
#include <iostream>
#include <exception>

void rawPrint(const std::string& input);

template<typename T>
void contentPrint(const std::string& print_head, const T& input){
    try{
        std::cout << "[" << print_head << "]:" << input << std::endl;
    }
    catch(std::exception& ex){
        std::cout << "Print error: " << ex.what() << std::endl;
    }
}

/**
 * @brief Filter function
 */
int sign(double input){
    if(input > 0){
        return 1;
    }
    else if (input < 0){
        return -1;
    }
    else
    {
        return 0;
    }
};

double sat(double input, double thick){                                                
    return fabs(input) >= thick ? sign(input) : (input / thick);
};


#endif
