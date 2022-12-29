#ifndef TRAJPLAN_H_
#define TRAJPLAN_H_


#include "frc/Filesystem.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdio.h>
#include <chrono>
#include <string>

class TalonXXIII;

//#defines
//example: #define LEVEL_ONE_HEIGHT     (23.0)

class TrajPlan
{
    private:
       // Create objects needed by this class
		// example: VictorSP *sampleMotor;

        TalonXXIII *mainRobot;
        std::vector<std::vector<double>> twoBall_Path1;
        std::vector<std::vector<double>> twoBall_Path2;
        std::vector<std::vector<double>> threeBall_Path1;
        std::vector<std::vector<double>> fourBall_Path1;
        std::vector<std::vector<double>> fourBall_Path2;
        // std::vector<std::vector<double>> fourBall_Path3;
        std::vector<std::vector<double>> fiveBall_Path1;
        std::vector<std::vector<double>> fiveBall_Path2;
        std::vector<std::vector<double>> twoBall_Path3;


        std::string path;
        std::string oldPath;


        //declare member variables
        //example: float height;

    public:
        TrajPlan(TalonXXIII* pRobot);
        //Functions
        void LocalReset(void);
        void Tokenize(std::string const &str, const char delim, std::vector<double> &out);
        bool ReadTrajFile(std::string const &fileName, std::vector<std::vector<double>> &pathVector);
        void ReadAllFiles(void);
        bool IsPathComplete(unsigned int index, unsigned int pathNum);
        std::vector<double> GetCmds(unsigned int index, unsigned int pathNum);
        void PrintAllPaths(void);
        void UpdateDash(void);
        void Service(void);
};
#endif /*Sample_H_*/