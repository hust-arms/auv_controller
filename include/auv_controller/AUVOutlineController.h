/*                                                                           
 * Filename: auv_controller/AUVOutlineController.h
 * Path: auv_controller
 * Created Date: Friday, April 17th 2021, 9:01:05 am
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#pragma once
#include <stdio.h>
#include <cstdio>
#include <stdlib.h>
#include <string>
#include <sqlite3.h>
#include <vector>

#include <boost/shared_ptr.hpp>

namespace auv_controller
{

struct AUVCtrlInfo
{
    /**
     * 1. for AUV with front fins in crosshead type:
     * fin0: front right fin
     * fin1: front left fin
     * fin2: back left fin
     * fin3: vertical upper fin
     * fin4: back right fin 
     * fin5: vertical lower fin
     * 2. for AUV without front fins in crosshead type:
     * fin0: back left fin
     * fin1: vertical upper fin
     * fin2: back right fin
     * fin3: vertical lower fin
     * 3. for AUV with fins in X type:
     * fin0: upper port fin
     * fin1: upper starboard fin
     * fin2: lower port fin
     * fin3: lower starboard fin
     */
    double fin0_, fin1_, fin2_, fin3_, fin4_, fin5_;
    double rpm_;
    double ts_;

    AUVCtrlInfo() : fin0_(0.0), fin1_(0.0), fin2_(0.0), fin3_(0.0), rpm_(0.0), ts_(0.0){}
}; // UAUVCtrlInfo

/**
 * @brief AUV type
 */
enum class AUVType
{
    WITH_FRONTFINS,
    COMMON,
    X_TYPE
}; // AUVType

/**
 * @brief AUV data base file parser
 */
class AUVDBParser
{
public:
    /**
     * @brief Constructor
     */
    AUVDBParser(const std::string& db_filename)
    {
        name_ = std::string("[AUVDBParser]: ");

        // Open database
        int db_ret = sqlite3_open(db_filename.c_str(), &db_);

        if(db_ret != SQLITE_OK)
        {
            fprintf(stderr, name_ + "Open database failed: %s\n", sqlite3_errmsg(db_));
            exit(1);
        }
        else
        {
            printf(name_ + "Open database successful: %s\n");
        }
    }

    ~AUVDBParser()
    {
        if(db_)
        {
            sqlite3_close(db_);
            db_ = nullptr;
        }
    }

    /**
     * @brief Parse data
     * @type Type of AUV
     */
    bool parse(AUVType type)
    {
        char** db_result;
        int nrow = 0, ncol = 0;
        char* er_msg;

        std::string select_query = "select * from PARAM";

        int db_ret = sqlite3_get_table(db_, select_query.c_str(), &db_result, &nrow, & ncol, &er_msg);

        if(db_ret != SQLITE_OK)
        {
            sqlite3_close(db_);
            fprintf(stderr, name_ + "Open database failed: %s\n", sqlite3_errmsg(db_));
            return false;
        }

        if(nrow && ncol)
        {
            for(int row = 0; row < nrow; ++row)
            {
                AUVCtrlInfo AUV_ctrl_info;
                
                // AUV with front fins
                if(type == AUVType::WITH_FRONTFINS)
                {
                    AUV_ctrl_info.ts_ = atof(db_result[row * ncol + 1]);
                    AUV_ctrl_info.rpm_ = atof(db_result[row * ncol + 30]);
                    AUV_ctrl_info.fin0_ = atof(db_result[row * ncol + 40]);
                    AUV_ctrl_info.fin1_ = atof(db_result[row * ncol + 39]);
                    AUV_ctrl_info.fin2_ = atof(db_result[row * ncol + 37]);
                    AUV_ctrl_info.fin3_ = atof(db_result[row * ncol + 35]);
                    AUV_ctrl_info.fin4_ = atof(db_result[row * ncol + 38]);
                    AUV_ctrl_info.fin5_ = atof(db_result[row * ncol + 36]);
                    
                    ctrl_info_list_.push_back();
                }

                // AUV without front fins in crosshead type
                if(flag == AUVType::COMMON)
                {

                }

                // AUV without front fins in X type
                if(flag == AUVType::X_TYPE)
                {

                }
            }
        }
        return true;
    }

    /**
     * @brief return parsed control information
     */
    std::vector<AUVCtrlInfo> getCtrlInfo()
    {
        return std::move(ctrl_info_list_);
    }
    

private:
    sqlite3* db_;
    std::string name_;

    std::vector<AUVCtrlInfo> ctrl_info_list_;
}; // AUVDBParser

/**
 * @brief AUV outline controller, parse control parameters from SQLite data base
 */
class AUVOutlineController
{
typedef boost::shared_ptr<AUVDBParser> parser_ptr;
public:
    /**
     * @brief Constructor
     */
    AUVOutlineController(const std::string& db_filename, bool with_ff, bool x_type)
    {
        // create database parser
        parser_ = boost::shared_ptr<AUVDBParser>(new AUVDBParser(db_filename));
    }

    /**
     * @brief Parse database according to type of AUV
     */
    bool parse()
    {
        if(with_ff)
        {
            return parser_->parse(AUVType::WITH_FRONTFINS);
        }

        if(x_type)
        {
            return parse_->parse(AUVType::X_TYPE);
        }

        return parse_->parse(AUVType::COMMON);
    }

    ~AUVOutlineController() {}

private:
    parser_ptr parser_;
}; // AUVOutlineController

}; //ns 
