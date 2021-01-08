//
// Created by mike on 18/12/20.
//

#include <iostream>
#include <regex>
#include <nlohmann/json.hpp>

using namespace std;
using namespace nlohmann;

nlohmann::json nop(nlohmann::json const & j) {
    return j.unflatten();
}

std::vector<std::string> split(const std::string str, const std::string regex_str)
{
    std::regex regexz(regex_str);
    std::vector<std::string> list(std::sregex_token_iterator(str.begin(), str.end(), regexz, -1),
                                  std::sregex_token_iterator());
    return list;
}

nlohmann::json read_jsonl(std::string const &s) {
    size_t idx = 0;
    std::string token;
//    std::vector<nlohmann::json> out;
    nlohmann::json out;
    auto lines = split(s, "\n");
    for (auto & line : lines) {
//        cout <<  ". : " << line << endl;
        try {
            out[idx++] = nlohmann::json::parse(line);
        } catch (nlohmann::detail::parse_error&) {
            cerr << "Not able to parse: " << line << endl;
        }
    }

    return out;
}

int main(int argc,char** argv)
{
    json j, k;
    j["pi"] = 3.141;
    j["answer"]["everything"] = 42;
    j["answer"]["nothing"] = "pain";
    k[0] = "foo";
    auto answer = j["answer"];
    string jraw = "{ \"happy\": true, \"pi\": 3.141 }\n{ \"things\": false, \"e\": 2.7}\n{dead}\n{\"end\": null}";
//    auto j3 = json::parse("{}");
    auto out = read_jsonl(jraw);

//    cout << j.dump() << endl;
//    cout << k.dump() << endl;
    cout << out.dump() << endl;
//    cout << answer.dump() << endl;
//    cout << j.flatten().dump() << endl;
//    cout << j3.dump() << endl;
}