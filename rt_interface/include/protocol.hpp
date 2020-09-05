//
// Created by inderjeet on 3/27/20.
//

#ifndef PROTOCOL_HPP_
#define PROTOCOL_HPP_

class Protocol {

 public:
  Protocol();
  ~Protocol();

  virtual bool pack_data();
  virtual bool unpack_data();

 private:
};

#endif //PROTOCOL_HPP_
