#include <string>
#include <vector>

#include <database_interface/db_class.h>

class returnConfiguration : public database_interface::DBClass
{
public:
  database_interface::DBField<int> id_;
  database_interface::DBField<int> robot_id_;

  returnConfiguration () :
    id_(database_interface::DBFieldBase::TEXT,this,"key_column","places2",true),
    robot_id_(database_interface::DBFieldBase::TEXT,this,"robot_id","places2",true)
  {
    primary_key_field_ = &id_;
    fields_.push_back(&robot_id_);
  }
};


