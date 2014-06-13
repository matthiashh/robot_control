#include <string>
#include <vector>
#include <database_interface/db_class.h>

class returnTasks : public database_interface::DBClass
{
public:
  database_interface::DBField<int> id_;
  database_interface::DBField<int> task_id_;
  database_interface::DBField<std::string> task_name_;
  database_interface::DBField<int> priority_;

  returnTasks() :
    id_(database_interface::DBFieldBase::TEXT,this,"key_column","places2",true),
    task_id_(database_interface::DBFieldBase::TEXT,this,"task_id","places2",true),
    task_name_(database_interface::DBFieldBase::TEXT,this,"task_name","places2",true),
    priority_(database_interface::DBFieldBase::TEXT,this,"priority","places2",true)
  {
    primary_key_field_ = &id_;
    fields_.push_back(&task_id_);
    fields_.push_back(&task_name_);
    fields_.push_back(&priority_);
  }
};


