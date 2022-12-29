#include "Common.h"
#include "TrajPlan.h"
#include "TalonXXIII_main.h"

using namespace std;

TrajPlan::TrajPlan(TalonXXIII* pRobot)
{
    mainRobot = pRobot;
    //LocalReset();
}

void TrajPlan::LocalReset()
{
    twoBall_Path1.clear();
    twoBall_Path2.clear();
    threeBall_Path1.clear();
    fourBall_Path1.clear();
    fourBall_Path2.clear();
    // fourBall_Path3.clear();
    fiveBall_Path1.clear();
    fiveBall_Path2.clear();
    twoBall_Path3.clear();
}

void TrajPlan::Tokenize(std::string const &str, const char delim, std::vector<double> &out)
{
    size_t start;
    size_t end = 0;
    while ((start = str.find_first_not_of(delim, end)) != string::npos)
    {
        end = str.find(delim, start);
        string temp= str.substr(start, end - start);
        out.push_back(stod(temp));
    }
}

bool TrajPlan::ReadTrajFile(std::string const &fileName, std::vector<std::vector<double>> &pathVector)
{
    string line;
    const char delim = ',';
    ifstream myfile (fileName);
    if (myfile.is_open())
    {
        while(!myfile.eof())
        {
            getline(myfile, line);
            vector<double> out;
            Tokenize(line, delim, out);
            pathVector.push_back(out);
        }
        myfile.close();
        return true;
    }
    return false;
} 

void TrajPlan::ReadAllFiles()
{
    twoBall_Path1.clear();
    twoBall_Path2.clear();
    threeBall_Path1.clear();
    fourBall_Path1.clear();
    fourBall_Path2.clear();
    // fourBall_Path3.clear();
    fiveBall_Path1.clear();
    fiveBall_Path2.clear();
    twoBall_Path3.clear();


    string directory = frc::filesystem::GetDeployDirectory();
    string fileName;

    fileName = "/2Ball_1";
    ReadTrajFile(directory + fileName, twoBall_Path1);

    fileName = "/2Ball_2";
    ReadTrajFile(directory + fileName, twoBall_Path2);

    fileName = "/3Ball_1";
    ReadTrajFile(directory + fileName, threeBall_Path1);

    fileName = "/4Ball_1";
    ReadTrajFile(directory + fileName, fourBall_Path1);

    fileName = "/4Ball_2";
    ReadTrajFile(directory + fileName, fourBall_Path2);

    // fileName = "/4Ball_3";
    // ReadTrajFile(directory + fileName, fourBall_Path3);

    fileName = "/5Ball_1";
    ReadTrajFile(directory + fileName, fiveBall_Path1);

    fileName = "/5Ball_2";
    ReadTrajFile(directory + fileName, fiveBall_Path2);

    fileName = "/2Ball_3";
    ReadTrajFile(directory + fileName, twoBall_Path3);
}

bool TrajPlan::IsPathComplete(unsigned int index, unsigned int pathNum)
{
    switch(pathNum)
    {
        case TWO_BALL_PATH_1:
            if (index < twoBall_Path1.size())
            {
                return false;
            }
            return true;
            break;

        case TWO_BALL_PATH_2:
            if (index < twoBall_Path2.size())
            {
                return false;
            }
            return true;
            break;

        case THREE_BALL_PATH_1:
            if (index < threeBall_Path1.size())
            {
                return false;
            }
            return true;
            break;

        case FOUR_BALL_PATH_1:
            if (index < fourBall_Path1.size())
            {
                return false;
            }
            return true;
            break;

        case FOUR_BALL_PATH_2:
            if (index < fourBall_Path2.size())
            {
                return false;
            }
            return true;
            break;

        // case FOUR_BALL_PATH_3:
        //     if (index < fourBall_Path3.size())
        //     {
        //         return false;
        //     }
        //     return true;
        //     break;

        case FIVE_BALL_PATH_1:
            if (index < fiveBall_Path1.size())
            {
                return false;
            }
            return true;
            break;

        case FIVE_BALL_PATH_2:
            if (index < fiveBall_Path2.size())
            {
                return false;
            }
            return true;
            break;

        case TWO_BALL_PATH_3:
            if (index < twoBall_Path3.size())
            {
                return false;
            }
            return true;
            break;
    }
}

std::vector<double> TrajPlan::GetCmds(unsigned int index, unsigned int pathNum) 
{
    switch(pathNum)
    {
        case TWO_BALL_PATH_1:
            return twoBall_Path1[index];
            break;

        case TWO_BALL_PATH_2:
            return twoBall_Path2[index];
            break;

        case THREE_BALL_PATH_1:
            return threeBall_Path1[index];
            break;

        case FOUR_BALL_PATH_1:
            return fourBall_Path1[index];
            break;

        case FOUR_BALL_PATH_2:
            return fourBall_Path2[index];
            break;

        // case FOUR_BALL_PATH_3:
        //     return fourBall_Path3[index];
        //     break;

        case FIVE_BALL_PATH_1:
            return fiveBall_Path1[index];
            break;

        case FIVE_BALL_PATH_2:
            return fiveBall_Path2[index];
            break;

        case TWO_BALL_PATH_3:
            return twoBall_Path3[index];
            break;

    }
}

void TrajPlan::PrintAllPaths()
{
    for(int i = 0; twoBall_Path1.size(); i++)
    {
        printf("%f, %f\n", twoBall_Path1[i][0], twoBall_Path1[i][1]);
    }
    printf("Done");

    // for(int i = 0; twoBall_Path2.size(); i++)
    // {
    //     printf("%f, %f\n", twoBall_Path2[i][0], twoBall_Path2[i][1]);
    // }
}