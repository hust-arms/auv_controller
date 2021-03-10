#ifndef COMMON_H_
#define COMMON_H_

#include <math.h>
#include <string>
#include <iostream>
#include <exception>

/**
 * @brief Factor yield number from -1 to 1 in circular period 
 */
class FlucFactor
{
public:
    FlucFactor();

    /**
     * @brief return fluctuate factor
     */
    double getFactor()
    {
        double f_copy = f;
        f = -f;
        return f_copy;
    };

private:
    static double f;
};

static double f = 1.0;

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


#endif
