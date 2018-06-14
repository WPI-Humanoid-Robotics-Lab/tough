#ifndef CONFIGURATIONREADER_H
#define CONFIGURATIONREADER_H

#include<iostream>
#include <algorithm>
#include <functional>
#include <cctype>
#include <locale>
#include <map>

class ConfigurationReader
{
public:
    ConfigurationReader();
    ConfigurationReader(const char*);
    char delimiter_;
    std::map<std::string, std::string> currentTopics;

private:
    void readConfiguration();
    void readConfiguration(const char*);
};


//Thanks Evan (http://stackoverflow.com/users/13430/evan-teran)
// trim from start
static inline std::string &ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
    return s;
}

// trim from end
static inline std::string &rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
    return s;
}

// trim from both ends
static inline std::string &trim(std::string &s) {
    return ltrim(rtrim(s));
}


#endif // CONFIGURATIONREADER_H
