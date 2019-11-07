//
// Created by ale on 19-11-7.
//

#ifndef MY3DPHOTO_0LOGGER_H
#define MY3DPHOTO_0LOGGER_H

#include<iostream>

namespace m3d
{

    class Logger
    {
    private:


    public:
        static std::string rootDir;
//        static//file pointer

        //initialization
        Logger(const std::string& rootDir_)
        {
            rootDir.assign(rootDir_);
            if(rootDir.empty() )
                return;
            //file operations
            //todo
        }

        //log
        void operator()(const std::string& label, const std::string& msg)
        {
            std::cout<<label<<"\t\t"<<msg<<std::endl;

            if(rootDir.empty())
                return;

            //file operations
            //todo
        }

        virtual ~Logger()
        {
            if(rootDir.empty())
                return;

            //file operation
            //todo
        }


    };

    std::string Logger::rootDir = std::string();
    Logger LOG("");

}

#endif //MY3DPHOTO_0LOGGER_H
