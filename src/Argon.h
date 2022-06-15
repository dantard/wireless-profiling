/*                Copyright (C) 2000-2017, Danilo Tardioli               *
 *           Centro Universitario de la Defensa Zaragoza, SPAIN          *
 *
 *  Contact Addresses: Danilo Tardioli                   dantard@unizar.es
 *
 *  This is free software; you can  redistribute it and/or  modify it
 *  under the terms of the GNU General Public License  as published by the
 *  Free Software Foundation;  either  version 2, or (at  your option) any
 *  later version.
 *
 *  This software is distributed  in the  hope  that  it will be   useful,
 *  but WITHOUT  ANY  WARRANTY;   without  even the implied   warranty  of
 *  MERCHANTABILITY  or  FITNESS FOR A  PARTICULAR PURPOSE.    See the GNU
 *  General Public License for more details.
 *
 *  You should have received  a  copy of  the  GNU General Public  License
 *  distributed with RT-WMP;  see file COPYING.   If not,  write to the
 *  Free Software  Foundation,  59 Temple Place  -  Suite 330,  Boston, MA
 *  02111-1307, USA.
 *
 *  As a  special exception, if you  link this  unit  with other  files to
 *  produce an   executable,   this unit  does  not  by  itself cause  the
 *  resulting executable to be covered by the  GNU General Public License.
 *  This exception does  not however invalidate  any other reasons why the
 *  executable file might be covered by the GNU Public License.
 *
 *----------------------------------------------------------------------*/

#ifndef ARGON_H_
#define ARGON_H_
#include <iomanip>
#include <iostream>
#include <cxxabi.h>
#include <getopt.h>
#include <yaml-cpp/yaml.h>

#define ARGON_ROS
#ifdef ARGON_ROS
#include<ros/ros.h>
#endif
class Argon{

    std::vector<option> ops;
    std::string opts;

    class Param{
    public:
        enum types_t {INT, FLOAT, STRING, BOOL};
    protected:
        types_t type;
        bool has_parameter, mandatory;
        std::string description, long_name;
        std::string hr_type;
        char name;
    public:
        void init(char name, const char * long_name, bool has_parameter, bool mandatory, const char * description){
            this->name = name;
            if (description!=0){
                this->description = std::string(description);
            }
            this->has_parameter = has_parameter;
            this->mandatory = mandatory;
            if (long_name!=0){
                this->long_name = std::string(long_name);
            }
        }
        char getName(){
            return name;
        }
        std::string getLongName(){
            return long_name;
        }
        std::string setLongName(std::string long_name){
            this->long_name = long_name;
        }
        std::string getDescription(){
            return description;
        }
        bool isMandatory(){
            return mandatory;
        }
        bool hasParameter(){
            return has_parameter;
        }
        virtual std::string getHRValue()=0;

        virtual std::string getHRType()=0;

        types_t getType(){
            return type;
        }
        virtual void setValue(std::string ){}
        virtual void processYAML(YAML::Node & config, bool override){}
    };

    template <class T> class GenParam :public Param{
        T & value;
        T dflt;
        bool * checker;
        void setHRType(){
            const std::type_info & t = typeid(T);
            if (t == typeid(int)
                    || t == typeid(unsigned int)
                    || t == typeid(char)
                    || t == typeid(unsigned char)
                    || t == typeid(short)
                    || t == typeid(unsigned short)
                    || t == typeid(long int)
                    || t == typeid(unsigned long long int)
                    || t == typeid(long long int)
                    || t == typeid(unsigned long int)){
                hr_type = "integer";
                type = INT;
            }else if (t == typeid(double) || t == typeid(float)){
                hr_type = "float  ";
                type = FLOAT;
            }else if (t == typeid(bool)){
                hr_type = "boolean";
                type = BOOL;
            }else if (t == typeid(std::string)){
                hr_type = "string ";
                type = STRING;
            }
        }
    public:
        GenParam(char name, const char* long_name, T & thevalue, T dflt, bool has_parameter, bool mandatory, const char * description, bool * checker): value(thevalue){
            this->dflt = dflt;
            this->value = dflt;
            this->checker = checker;
            if (checker != 0){
                *checker = false;
            }
            init(name, long_name, has_parameter, mandatory, description);
            setHRType();
        }
        virtual void setValue(std::string s){
            std::istringstream iss(s);
            iss >> value;
            if (checker){
                *checker = true;
            }
        }

        T & getValue(){
            return value;
        }

        virtual std::string getHRValue(){
            std::ostringstream oss;
            oss << name << " = " << std::left<< std::setw(20) << value;
            if (long_name.compare("")!=0) {
                oss << " (" << long_name << ")";
            }
            return oss.str();
        }

        std::string getHRType(){
            return hr_type;
        }

        T getDefaultValue(){
            return dflt;
        }
        void processYAML(YAML::Node & config, bool override){
            if (YAML::Node parameter = config[long_name]){
                if (value == dflt || override){
                    value = parameter.as<T>();
                }
            }
        }

    };
    std::vector<Param * > vec;
    int helper, maxlen;
    std::string example;
public:

    void showList(){
        std::cerr << "List of all options (* are mandatory):" << std::endl;
        for (int j = 0; j< vec.size(); j++){
            Param * p = vec[j];
            if (p->isMandatory()){
                std::cerr << "* ";
            }else{
                std::cerr << "  ";
            }
            std::cerr << "-" <<  p->getName();
            if (p->getLongName()!=""){
                std::cerr << ", --" << std::left<< std::setw(maxlen)  <<p->getLongName();
            }else{
                std::cerr << "    " << std::left<< std::setw(maxlen)  << "";
            }
            if (p->hasParameter()){
                std::cerr << " " << p->getHRType() << " : ";
            }else{
                std::cerr << "         : ";
            }

            std::cerr << p->getDescription() <<std::endl;
        }
        if (example.compare("")!=0){
            std::cerr << "Example: " << example << std::endl;
        }
    }

    template <class T> int add(char name, const char * long_name,T & value, T dflt, bool has_parameter, bool mandatory, const char * description, bool * provided){

        for (int i = 0; i< vec.size(); i++){
            if (vec[i]->getName()==name){
                std::cerr << "assertion failed: option '" << name <<"' is duplicated " <<std::endl;
                exit(1);
            }
        }

        GenParam<T> *p = new GenParam<T>(name, long_name, value, dflt, has_parameter, mandatory, description, provided);
        if (long_name != 0){
            struct option op;
            maxlen = strlen(long_name) > maxlen ? strlen(long_name) : maxlen;
            op.has_arg = has_parameter ? required_argument : no_argument;
            op.name  = long_name;
            op.val = 0; op.flag = 0;
            ops.push_back(op);
        }
        vec.push_back(p);
        opts += name;
        if(has_parameter){
            opts+=":";
        }
        return vec.size() - 1;
    }

    int addSwitch(char name, const char * long_name, bool & value, const char * description, bool * checker = 0){
        return add<bool>(name, long_name, value, false, false, false, description, checker);
    }

    int addToggle(char name, const char * long_name, bool & value, bool dflt, const char * description, bool * checker = 0){
        return add<bool>(name, long_name, value, dflt, false, false, description, checker);
    }

    int addSwitchInt(char name, const char * long_name, int & value, const char * description, bool * checker = 0){
        return add<int>(name, long_name, value, 0, false, false, description, checker);
    }
    int addSwitch(char name, bool & value, const char * description, bool * checker = 0){
        return add<bool>(name, 0, value, false, false, false, description, checker);
    }
    int addInt(char name, const char * long_name, int & value, int dflt, const char * description, bool * checker = 0){
        return add<int>(name, long_name, value, dflt, true, false, description, checker);
    }
    int addIntMandatory(char name, const char * long_name, int & value, int dflt, const char * description, bool * checker = 0){
        return add<int>(name, long_name, value, dflt, true, true, description, checker);
    }
    int addDouble(char name, const char * long_name, double & value, int dflt, const char * description, bool * checker = 0){
        return add<double>(name, long_name, value, dflt, true, false, description, checker);
    }
    int addDoubleMandatory(char name, const char * long_name, double & value, int dflt, const char * description, bool * checker = 0){
        return add<double>(name, long_name, value, dflt, true, true, description, checker);
    }
    int addString(char name, const char * long_name, std::string & value, std::string dflt, const char * description, bool * checker = 0){
        return add<std::string>(name, long_name, value, dflt, true, false, description, checker);
    }

    struct Relation{
        char option;
        int type;
        Relation(char option, int type){
            this->option = option;
            this->type = type;
        }
    };

    enum Direction {LEFT, RIGHT, BOTH};
    enum RelationType {DEPENDENCY, INCOMPATIBILITY, ALONE};
    std::map<char, std::vector<Relation> > relations;
    std::vector<int> switches;

#ifdef ARGON_ROS
    void processROS(){
        ros::NodeHandle nh("~");

        for (int j = 2; j< vec.size(); j++){
            Param * p = vec[j];
            bool used = false;
            std::string name = p->getLongName();
            std::replace(name.begin(), name.end(),'-','_');
            if (p->getType() == 0){
                GenParam<int> * p1 = (GenParam<int> *) p;
                int & value = p1->getValue();
                used = nh.getParam(name,value);
            }else  if (p->getType() == 1){
                GenParam<double> * p1 = (GenParam<double> *) p;
                double & value = p1->getValue();
                used = nh.getParam(name,value);
            }else  if (p->getType() == 2){
                GenParam<std::string> * p1 = (GenParam<std::string> *) p;
                std::string & value = p1->getValue();
                used = nh.getParam(name,value);
            }else  if (p->getType() == 3){
                GenParam<bool> * p1 = (GenParam<bool> *) p;
                bool & value = p1->getValue();
                bool has = nh.getParam(name,value);
                used = value && has;
            }

            if (used) {
                switches.push_back(j);
            }else{
                nh.deleteParam(name);
            }
        }
        process(0);
    }
#endif
    void process(int argc, char * argv[]){
        int verbose = parse(argc, argv);
        process(verbose);
    }

    int parse(int argc, char * argv[]){

        int verbose = 0;

        while (1) {
            int option_index = 0;
            int curind = optind;
            int c = getopt_long(argc, argv, opts.c_str(),
                                &ops[0], &option_index);
            if (c == -1){
                break;
            }else if(c == 'v'){
                verbose = 1;
                continue;
            }else if(c == 'h'){
                showList();
                exit(0);
            } else if(c == 'd'){
                for (int j = 2; j< vec.size(); j++){
                    std::cerr << vec[j]->getHRValue() << std::endl;
                }
                exit(1);
            }else if(c == '?'){
                std::cerr << "*** Unrecognized option '"<< argv[curind] << "'"<<std::endl;
                exit(1);
            }else if(c == ':'){
                std::cerr << "*** Missing parameter for option '" << argv[curind] << "'" <<std::endl;
                exit(1);
            }

            for (int i = 0; i< vec.size(); i++){
                Param * p = vec[i];
                if(p->getName() == c || (c==0 && p->getLongName().compare(ops[option_index].name) == 0)){
                    if (p->hasParameter()){
                        p->setValue(std::string(optarg));
                    }else{
                        GenParam<bool> * k = (GenParam<bool> *) p;
                        if (k->getDefaultValue()){
                            p->setValue("0");
                        }else{
                            p->setValue("1");
                        }
                    }
                    switches.push_back(i);
                    break;
                }
            }
        }
        return verbose;
    }

    void process(int verbose){
        /* Check mandatory switches */
        /* Vec contains all the DEFINED parameters, switches contiene los indices de los switches USED IN THIS EXECUTION referidos al vector vec */
        for (int j = 0; j< vec.size(); j++){
            Param * p = vec[j];
            if (p->isMandatory()){
                bool used = false;
                for (int i = 0; i<switches.size(); i++){
                    if (switches[i] == j){
                        used = true;
                    }
                }
                if (!used){
                    std::cerr << "*** Option '-" << vec[j]->getName() << "' is mandatory" << std::endl;
                    exit(0);
                }
            }
        }

//        /* Check boolean dependencies */
//        /* switches contiene los indices de los switches usados referidos al vector vec */
//        for (int j = 0; j< vec.size(); j++){
//            Param * p = vec[j];
//            char opt = p->getName();
//            // std::cerr << "*** opt << " << opt << std::endl;
//            if (cond.find(opt)!=cond.end()){
//                if ((*(cond[opt]))){
//                    std::cerr << "*** AAAOption '" << vec[j]->getName() << "' is mandatory" << std::endl;
//                    exit(0);
//                }
//            }
//        }

        /* check dependencies */
        for (std::map<char,std::vector<Relation> >::iterator it=relations.begin(); it!=relations.end(); ++it){
            std::string c1s = "", c2s = "";

            char c1 = it->first;
            std::vector<Relation> v = it->second;

            /* check if this switch has been used */
            bool used = false;
            for (int i = 0; i < switches.size(); i++){
                Param * param_used = vec[switches[i]];
                if (param_used->getName() == c1){
                    used = true;
                    c1s = param_used->getLongName();
                }
            }

            for (int j = 0; j < v.size(); j++){
                Relation r = v[j];
                bool found = false;
                for (int k = 0; k < switches.size(); k++){
                    Param * other_param_used = vec[switches[k]];
                    char c2 = other_param_used->getName();
                    if (r.option == c2){
                        found = true;
                        c2s = other_param_used->getLongName();
                    }
                }
                if (used && r.type == ALONE){
                    if (switches.size() > 1){
                        std::cerr << "*** Option '-" << c1 << "' ("<< c1s <<") must be used alone" << std::endl;
                        exit(0);
                    }
                }
                if (used && r.type == DEPENDENCY && !found){
                    std::cerr << "*** Option '-" << c1 << "' ("<< c1s <<") requires option '-" << r.option <<"'" << std::endl;
                    exit(0);
                }
                if (used && r.type == INCOMPATIBILITY && found){
                    std::cerr << "*** Option '-" << c1 << "' ("<< c1s <<") is incompatible with option '-" << r.option <<"' ("<< c2s <<")" << std::endl;
                    exit(0);
                }
            }
        }

        if (verbose){
            for (int j = 2; j< vec.size(); j++){
                std::cerr << vec[j]->getHRValue() << std::endl;
            }
        }
    }
    YAML::Node config;
    YAML::Node & processYAML(std::string & config_file, bool override = false){
        config = YAML::LoadFile(config_file);
        for (int j = 0; j< vec.size(); j++){
            vec[j]->processYAML(config, override);
        }
        return config;
    }

    bool provided(char c){
        for (int j = 0; j< vec.size(); j++){
            Param * p = vec[j];
            if (p->getName()==c){
                for (int i = 0; i<switches.size(); i++){
                    if (switches[i] == j){
                        return true;
                    }
                }
            }
        }
        return false;
    }

    void showValues(){
        for (int j = 2; j< vec.size(); j++){
            std::cerr << vec[j]->getHRValue() << std::endl;
        }

    }

    void addExample(std::string example){
        this->example = example;
    }

    void addDependency(char c1, char c2, Direction direction = RIGHT){
        if (direction == RIGHT){
            addRelation(c1,c2,DEPENDENCY);
        }else if (direction == LEFT){
            addRelation(c2,c1,DEPENDENCY);
        }else {
            addRelation(c1,c2,DEPENDENCY);
            addRelation(c2,c1,DEPENDENCY);
        }
    }

    void addDependency(char c1, const char * opts, Direction direction = RIGHT){
        for (int i = 0; i < strlen(opts); i++){
            addDependency(c1, opts[i], direction);
        }
    }

    void addIncompatibility(char c1, char c2){
        addRelation(c1,c2,INCOMPATIBILITY);
    }

    void addAlone(char c1){
        addRelation(c1,c1,ALONE);
    }

    void addIncompatibility(char c1, const char * opts){
        for (int i = 0; i < strlen(opts); i++){
            addIncompatibility(c1, opts[i]);
        }
    }

    void addRelation(char c1, char c2, int type){

        int dependent = -1;
        int master = -1;

        for (int i = 0; i<vec.size(); i++){
            if (vec[i]->getName() == c1){
                master = i;
            }
            if (vec[i]->getName() == c2){
                dependent = i;
            }
        }

        if (dependent == -1 || master == -1){
            std::cerr << "*** assertion failed: relation of '"<< c1 << "' with '"<< c2 <<"' involves inexistent switches" << std::endl;
            exit(1);
        }

        Relation r(c2,type);
        relations[c1].push_back(r);
    }

    void addRelation(char c1, const char * s, int type){
        for (int i = 0; i< strlen(s); i++){
            addRelation(c1, s[i], type);
        }
    }


    Argon(){
        maxlen = 0;
        opts = ":";
        add<int>('h', "help", helper,0, false, false, "Show this screen", 0);
        add<int>('v', "verbose",helper,0, false, false, "Show values assigned to parameters", 0);
        add<int>('d', "default",helper,0, false, false, "Show parameters' default values", 0);

    }
};

#define def_add_int(argo,param,param_long,var,dflt,desc) int var; argo.addInt(param, param_long, var, dflt, desc);
#define def_add_string(argo,param,param_long,var,dflt,desc) std::string var; argo.addString(param, param_long, var, dflt, desc);
#define def_add_switch(argo,param,param_long,var,desc) bool var; argo.addSwitch(param, param_long, var, desc);
#endif
