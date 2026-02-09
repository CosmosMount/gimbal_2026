#ifndef SLOPE_HPP
#define SLOPE_HPP

#include "math.hpp"

class SLOPE
{
protected:
    float _val;
    float _path;
    bool _reached;
public:
    SLOPE(float default_val, float path) : _val(default_val), _path(path), _reached(false) {}
    SLOPE() = default;
    inline void SetDefault(float val) 
    {
        _val = val;
    }

    inline void SetPath(float path) 
    {
        _path = path;
    }

    inline float GetVal() 
    {
        return _val;
    }
    inline float GetPath() 
    {
        return _path;
    }

    float UpdateVal(float new_val) 
    {
        if (fabsf(new_val - _val) < 1.0f * _path) 
        {
            _val = new_val;
            _reached = true;
        } 
        else 
        {
            _reached = false;
            if (new_val < _val) 
            {
                _val -= _path;
            } 
            else 
            {
                _val += _path;
            }
        }
        return _val;
    }

    bool CheckReached() 
    {
        return _reached;
    }
};

#endif