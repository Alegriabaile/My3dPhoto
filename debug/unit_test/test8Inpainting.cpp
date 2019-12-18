//
// Created by ale on 19-12-16.
//
#include <sys/time.h>
#include <sys/stat.h>
#include <unistd.h>
#include <iostream>
#include <fstream>


void createDir(const string dirName)
{
    if( 0 == access(dirName.c_str(), F_OK))
        cout<<"Path "<<dirName<<" exists"<<endl;
    else
    {
        state = mkdir(dirName.c_str(), fullAccess);
        if(state == 0)
            cout<<dirName<<" : mkdir succeed!"<<endl;
        else
            cout<<dirName<<" : mkdir failed!"<<endl;
    }
}
