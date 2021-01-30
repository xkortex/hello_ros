//
// Created by mike on 18/12/20.
//

#include <iostream>
#include <regex>
#include <stdio.h>
#include <time.h>
#include <nlohmann/json.hpp>
#include <boost/date_time.hpp>
#include <boost/filesystem.hpp>
//#include <atomic>


using namespace std;
using namespace nlohmann;

typedef std::pair<int, int> IntPair;

std::ostream& operator<<(std::ostream& os, IntPair p) {
    os << "(" << p.first << ", " << p.second << ")" ;
    return os;
}
std::ostream& operator<<(std::ostream& os, std::deque<IntPair> dq) {
    os << "[";
    for (const auto& i: dq) {
        os << i << ", " ;
    }
    os << "]";
    return os;
}
std::ostream& operator<<(std::ostream& os, std::deque<int> dq) {
    os << "[";
    for (const auto& i: dq) {
        os << i << ", " ;
    }
    os << "]";
    return os;
}
std::ostream& operator<<(std::ostream& os, std::vector<int> dq) {
    os << "[";
    for (const auto& i: dq) {
        os << i << ", " ;
    }
    os << "]";
    return os;
}

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

std::string now_micros()
{
    using boost::gregorian::date;
    using boost::posix_time::ptime;
    using boost::posix_time::microsec_clock;

    static ptime const epoch(date(1970, 1, 1));
    boost::posix_time::time_duration dt = microsec_clock::universal_time() - epoch;
    boost::posix_time::time_duration::tick_type dtm = dt.total_microseconds();
    auto total_seconds = dtm / 1000000;
    auto micros = dtm - (1000000 * total_seconds);
    std::string s_time = std::to_string(total_seconds);
    std::string s_micros = std::to_string(micros);
    s_time.append(".");
    for (auto i = 0; i < (6-s_micros.length()); i++) {
        s_time.append("_");
    }
    s_time.append(s_micros);
    std::cerr << "dtm  : " << dtm << std::endl;
    std::cerr << "time: " << s_time << std::endl;
    return s_time;
}

int main(int argc,char** argv)
{
    json j, k;
    j["pi"] = 3.141;
    j["answer"]["everything"] = 42;
    j["answer"]["nothing"] = "pain";
    k[0] = "foo";
    auto answer = j["answer"];
    string jraw = "{ \"happy\": true, \"pi\": 3.141 }\n{ \"things\": false, \"e\": 2.7}\n{\"end\": null}";
//    auto j3 = json::parse("{}");
    auto out = read_jsonl(jraw);

//    cout << j.dump() << endl;
//    cout << k.dump() << endl;
    cout << out.dump() << endl;
//    cout << answer.dump() << endl;
//    cout << j.flatten().dump() << endl;
//    cout << j3.dump() << endl;

    auto thing = j["nope"]["nuhuh"];
    json stringy{"only string"};
    json stringy2 = json::parse("\"null\"");
    json stringy3 = "stringy3";
    cout << thing.dump() << " : " << thing.is_primitive() << thing.is_null() << endl;
    cout << stringy.dump() << " : " << stringy.is_primitive() << endl;
    cout << stringy2.dump() << " : " << stringy2.is_primitive() << endl;
    cout << stringy3.dump() << " : " << stringy3.is_primitive() << string(stringy3) << endl;
    json dummy = j["does not exist"];
    cout << dummy.dump() << " " << dummy.is_null() << endl;
//    boost::filesystem::path pth{"/tmp/home/mike/tmp"};
//    boost::filesystem::path dirname = pth.parent_path();
//    boost::filesystem::create_directories(dirname);
    json testint = json::parse(R"({"foo": 42, "bar": "42", "spam": 42.5, "nope": {"nuhuh": "maybe"}})");
    cout << "testint: " << testint  << testint["foo"].is_number() << endl;
    string s_x = testint["bar"].get_ref<std::string &>(); //get_to(nlohmann::json::value_t())
    int s_y = testint["foo"]; //get_to(nlohmann::json::value_t())
    int x = std::stoi(s_x, 0, 10);
    cout << x + 1 << endl;
//    json bob{R"({"spam": true})"};
    json bob;
    bob["spam"] = true;
    json spam = bob["spam"];
    cout << bob.dump() << spam.dump() << spam.is_boolean() << spam.is_number() << endl;
    json jtrue = json::parse("2");
    double jtval = jtrue.front();

    cout << jtrue.dump() << jtval << endl;
    cout << "time: " << now_micros() << endl;
    j.update(testint);
    cout << j.dump() << "" << endl;
    cout << std::string(j["bar"]) << "" << endl;


    IntPair {5,7};
    std::deque<IntPair > dq;
    std::deque<IntPair > rdq;
    std::deque<int > idq;
    std::deque<int > odq;
    int val = 8;
    auto isVal = [val](IntPair p) {
        return p.first == val;
    };
    auto isOdd = [](IntPair p) {
        return p.first&1;
    };

    idq.emplace_back(5);
    idq.emplace_back(6);
    idq.emplace_back(7);
    idq.emplace_back(8);
    dq.emplace_back(IntPair{5,5});
    dq.emplace_back(IntPair{6,6});
    dq.emplace_back(IntPair{7,7});
    dq.emplace_back(IntPair{8,8});
    cout << "idq start " << idq << endl;

//    auto it = remove_if(dq.begin(), dq.end(), isVal);
//    cerr << *it << endl;

    odq.resize(idq.size());
    std::copy_if(idq.begin(), idq.begin() + idq.size(), odq.begin(), [](int i) {return i==6;});
//    auto remover = std::remove_copy_if1(dq.begin(), dq.end(), rdq.begin(), isOdd);
    for (auto it = odq.begin(); it!=odq.end(); ++it) {
        cerr <<  "?: " << *it << endl;
    }
    cerr << "odq " << odq << endl;
//    dq.erase(remover, dq.end());
//    for (auto it = dq.begin(); it != dq.end(); ++it) {
//        if (it->first == 8) {
//            dq.erase(it);
//        }
//    }
//    for (auto it = dq.begin(); it != dq.end(); ++it) {
//        if (it->first == 8) {
//            dq.erase(it);
//        }
//    }
    cout << "idq end" << idq << endl;
//    for (auto el : dq) {
//        cout << el  << ", " ;
//    }
    cout << "done" << endl;


}