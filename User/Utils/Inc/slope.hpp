#ifndef SLOPE_HPP
#define SLOPE_HPP

#include "math.hpp"

class SLOPE
{
protected:
    float _val;
    float _increase_path;
    float _decrease_path;
    bool _reached;
public:
    SLOPE(float default_val, float path) : _val(default_val), _reached(false) 
    {
        _increase_path = path;
        _decrease_path = path;
    }
    SLOPE() = default;
    inline void SetDefault(float val) 
    {
        _val = val;
    }

    inline void SetPath(float path) 
    {
        _increase_path = path;
        _decrease_path = path;
    }

    inline void SetIncreasePath(float path)
    {
        _increase_path = path;
    }

    inline void SetDecreasePath(float path)
    {
        _decrease_path = path;
    }

    inline float GetVal() 
    {
        return _val;
    }

    float UpdateVal(float new_val) 
    {
        float delta = new_val - _val;
        if ((delta>=0.0f&&delta<_increase_path) || (delta<0.0f&&delta>-_decrease_path)) 
        {
            _val = new_val;
            _reached = true;
        } 
        else 
        {
            _reached = false;
            if (new_val < _val) 
            {
                _val -= _decrease_path;
            } 
            else 
            {
                _val += _increase_path;
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