//
// Created by mike on 18/12/20.
//

#include <iostream>
#include <nlohmann/json.hpp>

using namespace std;
using namespace nlohmann;

nlohmann::json nop(nlohmann::json const & j) {
    return j.unflatten();
}

int main(int argc,char** argv)
{
    json j;
    j["pi"] = 3.141;
    j["answer"]["everything"] = 42;
    j["answer"]["nothing"] = "pain";
    auto answer = j["answer"];
    cout << j.dump() << endl;
    cout << answer.dump() << endl;
    cout << j.flatten().dump() << endl;
}