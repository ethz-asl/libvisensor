/*
 * Copyright (c) 2014, Skybotix AG, Switzerland (info@skybotix.com)
 * Copyright (c) 2014, Autonomous Systems Lab, ETH Zurich, Switzerland
 *
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef PARAM_SERVER_HPP_
#define PARAM_SERVER_HPP_

#include <config/config.hpp>
#include <map>

namespace param_server {

enum ParamTypes {
  BOOL_T,
  FLOAT_T,
  UINT_T,
  INT_T,
  CONST_T
};

class AbstractParamItem {
 public:
  AbstractParamItem()
      : register_(0),
        bit_shift_(0),
        param_name_("") {
  }
  ;
  virtual ~AbstractParamItem() {
  }
  ;

  virtual uint16_t getValue()=0;
  virtual bool setValue(uint16_t value)=0;
  virtual uint16_t getRegValue(uint16_t prevRegValue)=0;
  uint16_t getRegister() {
    return register_;
  }
  ;

 protected:
  uint16_t countTrailingZeros(uint16_t x) {
    if (x == 0)
      return 0;
    uint16_t r = 0;
    while ((x & 1) == 0) {
      x = x >> 1;
      r++;
    }
    return r;
  }
  ;

  uint16_t countLeadingZeros(uint16_t x) {
    if (x == 0)
      return 0;
    uint16_t r = 0;
    const uint16_t mask = 1 << (sizeof(uint16_t) * 8 - 1);  // 0x8000 for uint16_t
    while ((x & mask) == 0) {
      x = x << 1;
      r++;
    }
    return r;
  }
  ;

  uint16_t register_;
  uint16_t bit_shift_;
  std::string param_name_;
};

class ParamItemUInt : public AbstractParamItem {
 public:
  ParamItemUInt(uint16_t reg, uint16_t def, uint16_t mask) {
    _mask = mask;
    _default = def;
    _value = _default;
    AbstractParamItem::register_ = reg;
    AbstractParamItem::bit_shift_ = AbstractParamItem::countTrailingZeros(
        _mask);

    calcDefaultMinMax();

    //VISENSOR_DEBUG("max %u %u\n",tempNumOfBits,_max);
  }
  ;
  ParamItemUInt(uint16_t reg, uint16_t def, uint16_t mask, uint16_t minVal,
                uint16_t maxVal) {
    _mask = mask;
    _default = def;
    _value = _default;
    AbstractParamItem::register_ = reg;
    AbstractParamItem::bit_shift_ = AbstractParamItem::countTrailingZeros(
        _mask);

    calcDefaultMinMax();

    if (minVal < _min)
      VISENSOR_DEBUG("MINIMUM %u too small for desired parameter (min %u)!!!\n",
                     minVal, _min);
    else
      _min = minVal;
    if (maxVal > _max)
      VISENSOR_DEBUG("MAXIMUM %u too big for desired parameter (max %u)!!!\n",
                     maxVal, _max);
    else
      _max = maxVal;
  }
  virtual ~ParamItemUInt() {
  }
  ;

  virtual uint16_t getValue() {
    return _value;
  }
  ;

  virtual bool setValue(uint16_t value) {
    _value = cropVal(value);
    return true;
  }
  ;

  virtual uint16_t getRegValue(uint16_t prevRegValue) {
    uint16_t newRegValue = _value << AbstractParamItem::bit_shift_;
    //VISENSOR_DEBUG("trailing zeros: %u leading zeros %u",AbstractParamItem::countTrailingZeros(_mask),AbstractParamItem::countLeadingZeros(_mask));
    return (prevRegValue & (~_mask)) | (newRegValue & _mask);
  }
  ;

 private:
  void calcDefaultMinMax() {
    _min = 0;
    uint16_t tempNumOfBits = sizeof(uint16_t) * 8
        - AbstractParamItem::countLeadingZeros(_mask)
        - AbstractParamItem::countTrailingZeros(_mask);
    _max = (1 << tempNumOfBits) - 1;
  }
  ;

  uint16_t cropVal(uint16_t val) {
    if (val < _min) {
      VISENSOR_DEBUG("desired value %u too small\n", val);
      return _min;
    }
    if (val > _max) {
      VISENSOR_DEBUG("desired value %u too big\n", val);
      return _max;
    }
    return val;
  }
  ;

  uint16_t _mask;
  uint16_t _default;
  uint16_t _min;
  uint16_t _max;
  uint16_t _value;

};

class ParamItemInt : public AbstractParamItem {
 public:
  ParamItemInt(uint16_t reg, uint16_t def, uint16_t mask) {
    mask_ = mask;
    uint16_t * temp = (uint16_t*) &def;
    default_ = *temp;
    value_ = default_;
    AbstractParamItem::register_ = reg;
    AbstractParamItem::bit_shift_ = AbstractParamItem::countTrailingZeros(
        mask_);

    calcDefaultMinMax();

    //VISENSOR_DEBUG("max %u %u\n",tempNumOfBits,_max);
  }
  ;

  ParamItemInt(uint16_t reg, uint16_t def, uint16_t mask, uint16_t minVal,
               uint16_t maxVal) {
    mask_ = mask;
    default_ = def;
    value_ = default_;
    AbstractParamItem::register_ = reg;
    AbstractParamItem::bit_shift_ = AbstractParamItem::countTrailingZeros(
        mask_);

    calcDefaultMinMax();

    if (minVal < min_)
      VISENSOR_DEBUG("MINIMUM %u too small for desired parameter (min %i)!!!\n",
                     minVal, min_);
    else
      min_ = minVal;
    if (maxVal > max_)
      VISENSOR_DEBUG("MAXIMUM %u too big for desired parameter (max %i)!!!\n",
                     maxVal, max_);
    else
      max_ = maxVal;
  }

  virtual ~ParamItemInt() {
  }
  ;

  virtual uint16_t getValue() {
    uint16_t * temp = (uint16_t*) &value_;
    return *temp;
  }
  ;
  virtual bool setValue(uint16_t value) {
    int16_t * temp = (int16_t*) &value;
    value_ = cropVal(*temp);
    return true;
  }
  ;

  virtual uint16_t getRegValue(uint16_t prevRegValue) {
    uint16_t * temp = (uint16_t*) &value_;
    uint16_t newRegValue = *temp << AbstractParamItem::bit_shift_;

    //VISENSOR_DEBUG("trailing zeros: %u leading zeros %u",AbstractParamItem::countTrailingZeros(_mask),AbstractParamItem::countLeadingZeros(_mask));

    return (prevRegValue & (~mask_)) | (newRegValue & mask_);
  }
  ;

 private:
  void calcDefaultMinMax() {
    uint16_t tempNumOfBits = sizeof(uint16_t) * 8
        - AbstractParamItem::countLeadingZeros(mask_)
        - AbstractParamItem::countTrailingZeros(mask_);
    min_ = 0 - (1 << (tempNumOfBits - 1));
    max_ = (1 << (tempNumOfBits - 1)) - 1;
  }
  ;

  int16_t cropVal(int16_t val) {
    if (val < min_) {
      VISENSOR_DEBUG("desired value %u too small\n", val);
      return min_;
    }
    if (val > max_) {
      VISENSOR_DEBUG("desired value %u too big\n", val);
      return max_;
    }
    return val;
  }
  ;

  uint16_t mask_;
  int16_t default_;
  int16_t min_;
  int16_t max_;
  int16_t value_;
};

class ParamItemBool : public AbstractParamItem {
 public:
  ParamItemBool(uint16_t reg, uint16_t def, uint16_t mask) {
    _mask = mask;
    _default = (def & 1);
    _value = _default;
    AbstractParamItem::register_ = reg;
    AbstractParamItem::bit_shift_ = AbstractParamItem::countTrailingZeros(
        _mask);
  }
  ;
  virtual ~ParamItemBool() {
  }
  ;
  virtual uint16_t getValue() {
    return _value;
  }
  ;
  virtual bool setValue(uint16_t value) {
    _value = value & 1;
    return true;
  }
  ;
  virtual uint16_t getRegValue(uint16_t prevRegValue) {
    uint16_t newRegValue = _value << AbstractParamItem::bit_shift_;

    //VISENSOR_DEBUG("trailing zeros: %u leading zeros %u",AbstractParamItem::countTrailingZeros(_mask),AbstractParamItem::countLeadingZeros(_mask));

    return (prevRegValue & (~_mask)) | (newRegValue & _mask);
  }
  ;
 private:
  uint16_t _mask;
  bool _default;
  bool _value;
};

class ParamItemConst : public AbstractParamItem {
 public:
  ParamItemConst(uint16_t reg, uint16_t def, uint16_t mask) {
    _mask = mask;
    _default = def;
    _value = _default;
    AbstractParamItem::register_ = reg;
    AbstractParamItem::bit_shift_ = AbstractParamItem::countTrailingZeros(
        _mask);

  }
  ;
  virtual ~ParamItemConst() {
  }
  ;
  virtual uint16_t getValue() {
    return _value;
  }
  ;
  virtual bool setValue(uint16_t value) {
    if (value != _default)
      VISENSOR_DEBUG(
          "PARAM_SERVER.HPP : ERROR - trying to change constant value");
    return false;
    //_value=value&1;
  }
  ;
  virtual uint16_t getRegValue(uint16_t prevRegValue) {
    uint16_t newRegValue = _value << AbstractParamItem::bit_shift_;

    //VISENSOR_DEBUG("trailing zeros: %u leading zeros %u",AbstractParamItem::countTrailingZeros(_mask),AbstractParamItem::countLeadingZeros(_mask));

    return (prevRegValue & (~_mask)) | (newRegValue & _mask);
  }
  ;
 private:
  uint16_t _mask;
  uint16_t _default;
  uint16_t _value;
};

class ParamServer {
 public:
  void addParam(const std::string paramName, const ParamTypes type,
                const uint16_t reg, const uint16_t def, const uint16_t mask) {
    // check if parameter name exists
    if (_params.count(paramName)) {
      VISENSOR_DEBUG("Parameter \"%s\" already exists!!!\n", paramName.c_str());
      return;
    }

    if (type == BOOL_T) {
      ParamItemBool * newParam = new ParamItemBool(reg, def, mask);
      _params[paramName] = newParam;
      //newParam->getValue(0x08,0x04);
    }

    if (type == UINT_T) {
      ParamItemUInt * newParam = new ParamItemUInt(reg, def, mask);
      _params[paramName] = newParam;
      //newParam->getValue(0x08,0x04);
    }

    if (type == INT_T) {
      ParamItemInt * newParam = new ParamItemInt(reg, def, mask);
      _params[paramName] = newParam;
      //newParam->getValue(0x08,0x04);
    }

    if (type == CONST_T) {
      ParamItemConst * newParam = new ParamItemConst(reg, def, mask);
      _params[paramName] = newParam;
      //newParam->getValue(0x08,0x04);
    }

    uint16_t prevValue = 0;

    // if register already exists, load previously stored value
    if (_registerValues.count(reg)) {
      //VISENSOR_DEBUG("Register %#X already exists\n",reg);
      prevValue = _registerValues[reg];
    }

    _registerValues[reg] = _params[paramName]->getRegValue(prevValue);

    //VISENSOR_DEBUG("new register value: %#X\n",_registerValues[reg]);
  }
  ;

  // TODO find better way to deal with min max
  void addParam(const std::string paramName, const ParamTypes type,
                const uint16_t reg, const uint16_t def, const uint16_t mask,
                const uint16_t minVal, const uint16_t maxVal) {
    // check if parameter name exists
    if (_params.count(paramName)) {
      VISENSOR_DEBUG("Parameter \"%s\" already exists!!!\n", paramName.c_str());
      return;
    }

    if (type == BOOL_T) {
      ParamItemBool * newParam = new ParamItemBool(reg, def, mask);
      _params[paramName] = newParam;
      //newParam->getValue(0x08,0x04);
    }

    if (type == UINT_T) {
      ParamItemUInt * newParam = new ParamItemUInt(reg, def, mask, minVal,
                                                   maxVal);
      _params[paramName] = newParam;
      //newParam->getValue(0x08,0x04);
    }

    if (type == INT_T) {
      ParamItemInt * newParam = new ParamItemInt(reg, def, mask);  //TODO implement min max
      _params[paramName] = newParam;
      //newParam->getValue(0x08,0x04);
    }

    if (type == CONST_T) {
      ParamItemConst * newParam = new ParamItemConst(reg, def, mask);
      _params[paramName] = newParam;
      //newParam->getValue(0x08,0x04);
    }

    uint16_t prevValue = 0;

    // if register already exists, load previously stored value
    if (_registerValues.count(reg)) {
      //VISENSOR_DEBUG("Register %#X already exists\n",reg);
      prevValue = _registerValues[reg];
    }

    _registerValues[reg] = _params[paramName]->getRegValue(prevValue);

    //VISENSOR_DEBUG("new register value: %#X\n",_registerValues[reg]);
  }
  ;

  bool getConfigParam(const std::string paramName, const uint16_t value,
                      uint16_t &reg, uint16_t &regVal) {
    // check if parameter name exists
    if (!_params.count(paramName)) {
      VISENSOR_DEBUG("Parameter \"%s\" does not exist!!!\n", paramName.c_str());
      return false;
    }

    reg = _params[paramName]->getRegister();

    // check if parameter was changed
    if (_params[paramName]->getValue() == value) {
      regVal = _registerValues[reg];
      return false;
    }

    uint16_t prevValue = _registerValues[reg];

    _params[paramName]->setValue(value);
    regVal = _params[paramName]->getRegValue(prevValue);

    _registerValues[reg] = regVal;
    return true;
  }
  ;

  bool getConfigParam(const std::string paramName, uint16_t &reg,
                      uint16_t &regVal) {
    // check if parameter name exists
    if (!_params.count(paramName)) {
      VISENSOR_DEBUG("Parameter \"%s\" does not exist!!!\n", paramName.c_str());
      return false;
    }

    reg = _params[paramName]->getRegister();

    uint16_t prevValue = _registerValues[reg];

    regVal = _params[paramName]->getRegValue(prevValue);

    _registerValues[reg] = regVal;
    return true;
  }
  ;

 private:

  std::map<std::string, AbstractParamItem*> _params;
  std::map<uint16_t, uint16_t> _registerValues;
};

}  // namespace param_server

#endif /* PARAM_SERVER_HPP_ */
