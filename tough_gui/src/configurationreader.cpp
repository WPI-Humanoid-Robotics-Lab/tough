#include "tough_gui/configurationreader.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <typeinfo>
#include <string>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>


ConfigurationReader::ConfigurationReader()
{
    delimiter_ = '=';
}

ConfigurationReader::ConfigurationReader(const char *file_name)
{
    readConfiguration(file_name);
}

void ConfigurationReader::readConfiguration(){
    return readConfiguration("config.ini");
}

void ConfigurationReader::readConfiguration(const char *file_name){

    std::string current_line,token;
    std::cout<<file_name<<std::endl;
    std::ifstream config_file(file_name);
    std::string topicName;
    std::string topicType;
    bool isValue = false;

    if(config_file.is_open()){
        while(std::getline(config_file,current_line)){
            current_line = trim(current_line);
            if(current_line.substr(0,1)=="#")
                continue;
            boost::char_separator<char> token("=");
            boost::tokenizer<boost::char_separator<char> > tokens(current_line, token);
            BOOST_FOREACH(std::string t, tokens)
            {
                if(isValue){
                    currentTopics[topicType] = trim(t);
                    isValue=false;
                }
                else{
                    topicType =  trim(t);
                    isValue = true;
                }
            }
        }
        config_file.close();
    }
    else
        std::cout<<"Could not read the file"<<std::endl;
}
