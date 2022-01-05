// For logging
#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>

class Logger
{
public:
  Logger(){
    work_dir_="/home/leecw/Logs";
    log_trajectory_ptr_ = nullptr;
    log_metric_ptr_ = nullptr;
  }

  Logger(std::string work_dir){
    work_dir_=work_dir;
  }

  void register_test_name(std::string test_name)
  {
    test_dir_ = work_dir_ + "/" + test_name;
    std::string command = "mkdir -p " + test_dir_;
    system(command.data());
  }

  void clear()
  {
    if (log_trajectory_ptr_ != nullptr)
    {
      log_trajectory_ptr_->close();
      log_metric_ptr_->close();
      delete log_trajectory_ptr_;
      delete log_metric_ptr_;
      std::cout << "[Logger/clear()] -- success" << std::endl;
    }
  }

  void begin_new_test()
  {
    ++i_test;
    clear();
    std::string file_name1 = test_dir_ + "/traj_" + std::to_string(i_test) + ".txt";
    std::string file_name2 = test_dir_ + "/nums_" + std::to_string(i_test) + ".txt";
    log_trajectory_ptr_ = new std::ofstream(file_name1);
    log_metric_ptr_ = new std::ofstream(file_name2);
    std::cout << "[Logger] -- Write on \n\t" << file_name1 << " and \n\t" << file_name2 << std::endl;
  }

  ~Logger()
  {
    clear();
  }

  void add_points(const double & x, const double & y)
  {
    if (log_trajectory_ptr_->is_open())
    {
        (*log_trajectory_ptr_) << x;
        (*log_trajectory_ptr_) << ",";
        (*log_trajectory_ptr_) << y;
        (*log_trajectory_ptr_) << std::endl;
    }
    else
    {
      std::cout << "[Logger] -- traj file is not opened!!" << std::endl;
    }
  }

  void add_nums(const double & x, const double & y)
  {

    if (log_metric_ptr_->is_open())
    {
        (*log_metric_ptr_) << "Something" << std::endl;
    }
    else
    {
      std::cout << "[Logger] -- nums file is not opened!!" << std::endl;
    }
  }


private:
  std::string work_dir_;
  std::string test_dir_;
  int i_test=-1;
  std::ofstream* log_trajectory_ptr_;
  std::ofstream* log_metric_ptr_;
};
