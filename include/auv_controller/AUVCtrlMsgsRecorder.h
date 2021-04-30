/*                                                                                             
 * Filename: auv_controller/AUVCtrlMsgsRecorder.h
 * Path: auv_controller
 * Created Date: Thirsday, March 24th 2021, 10:58:05 am
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#ifndef AUV_CTRLMSGS_RECORDER_H_
#define AUV_CTRLMSGS_RECORDER_H_

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/chrono.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "file_writer.h"

namespace auv_controller
{
typedef boost::shared_ptr<FileWriter> FileWriterPtr;

/**
 * @brief AUV control messages recorder 
 */
class AUVCtrlMsgsRecorder
{
public:
    /**
     * @brief Constructor
     */
    AUVCtrlMsgsRecorder(const std::string& filename, const std::string& path) 
    {
        fw_ptr_ = boost::make_shared<FileWriter>(filename, path);
    }

    /**
     * @brief Constructor
     */
    AUVCtrlMsgsRecorder(const std::string& filename) 
    {
        fw_ptr_ = boost::make_shared<FileWriter>(filename);
    }

    /**
     * @brief Deconstructor
     */
    ~AUVCtrlMsgsRecorder() {}

    /**
     * @brief Write message head
     */
    bool writeMsgsHeader(const unsigned int model_type, const unsigned int mission_type);

    /**
     * @brief Write data into file
     */
    bool ctrlMsgsRecord(const std::vector<double>& data_arr);

    /**
     */
    bool isRecordOutlineParams()const
    {
        return is_record_ot_params_;
    }

private:
    /**
     * @brief Serialize control messages
     */
    std::string serializeData(const std::vector<double>& data_arr);

private:
    FileWriterPtr fw_ptr_;

    std::string filename_, path_;

    bool is_record_ot_params_;

    const unsigned int ctrl_msgs_num_ = 31;

}; // class AUVCtrlMsgsRecorder
}; // ns
#endif
