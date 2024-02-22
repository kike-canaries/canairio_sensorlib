
// ----------------------------------------------------------------------------------------------------------------------------------------
// MovingSum - template class for a moving-sum evaluation
// ----------------------------------------------------------------------------------------------------------------------------------------
// derived from https://github.com/alphaville/MovingAverage

#ifndef MOVING_SUM_H
#define MOVING_SUM_H

#define DEFAULT_FILTER_LENGTH 8  // default length of the filter

// ----------------------------------------------------------------------------------------------------------------------------------------
// template types are: <data_type>, <sum_type>
//
// <data_type>    : should be an integer type large enough to fit the current sample variable
// <sum_type>     : must be large enough to contain the sum of filter_length samples, each of type
// <data_type>
//
// example: MovingSum adc<uint8_t,  uint16_t>
//          MovingSum adc<uint16_t, uint32_t>
//          MovingSum adc<uint16_t, uint32_t>
// ----------------------------------------------------------------------------------------------------------------------------------------

template <class MA_dt, class MA_st>
class MovingSum {
 public:
  MovingSum(unsigned short _filter_length = DEFAULT_FILTER_LENGTH);
  ~MovingSum();

  void add(MA_dt x);
  void clear(void);
  MA_dt* getData(void);
  MA_st getCurrentSum(void) const;
  unsigned short getFilterLength(void) const;
  unsigned short getCurrentFilterLength(void) const;

 private:
  MA_st sum;                     // sum of current samples
  MA_dt* data;                   // vector with raw data
  unsigned short index;          // index of current sample
  unsigned short filter_length;  // length of the filter
  bool filter_complete;          // true when there are filter_length samples

  void init();
};

// ----------------------------------------------------------------------------------------------------------------------------------------
// constructor - Creates a new instance of MovingSum with a default or a given filter length.
// ----------------------------------------------------------------------------------------------------------------------------------------

template <class MA_dt, class MA_st>
MovingSum<MA_dt, MA_st>::MovingSum(unsigned short _filter_length) {
  filter_length = _filter_length;

  init();
}

// ----------------------------------------------------------------------------------------------------------------------------------------
// destructor - releases the memory objects associated with the current MovingSum instance.
// ----------------------------------------------------------------------------------------------------------------------------------------

template <class MA_dt, class MA_st>
MovingSum<MA_dt, MA_st>::~MovingSum() {
  delete[] data;
}

// ----------------------------------------------------------------------------------------------------------------------------------------
// init - initialize the class and allocate the required memory space
// ----------------------------------------------------------------------------------------------------------------------------------------

template <class MA_dt, class MA_st>
void MovingSum<MA_dt, MA_st>::init(void) {
  sum = 0;
  index = -1;
  filter_complete = false;

  data = new MA_dt[filter_length];

  clear();
}

// ----------------------------------------------------------------------------------------------------------------------------------------
// clear - clears the vector of data by setting it to zero.
// ----------------------------------------------------------------------------------------------------------------------------------------

template <class MA_dt, class MA_st>
void MovingSum<MA_dt, MA_st>::clear(void) {
  for (unsigned short i = 0; i < filter_length; i++) {
    data[i] = 0;
  }
}

// ----------------------------------------------------------------------------------------------------------------------------------------
// add - adds a new element in the Moving Sum vector. Updates the current sum.
// ----------------------------------------------------------------------------------------------------------------------------------------

template <class MA_dt, class MA_st>
void MovingSum<MA_dt, MA_st>::add(MA_dt x) {
  index = (index + 1) % filter_length;
  sum -= data[index];
  data[index] = x;
  sum += x;

  if (!filter_complete && index == filter_length - 1) {
    filter_complete = true;
  }
}

// ----------------------------------------------------------------------------------------------------------------------------------------
// getCurrentSum - returns the current sum as updated after the invocation of MovingSum::add(MA_dt).
// ----------------------------------------------------------------------------------------------------------------------------------------

template <class MA_dt, class MA_st>
MA_st MovingSum<MA_dt, MA_st>::getCurrentSum(void) const {
  return sum;
}

// ----------------------------------------------------------------------------------------------------------------------------------------
// getData - returns the raw data that are currently stored in an internal vector.
// ----------------------------------------------------------------------------------------------------------------------------------------

template <class MA_dt, class MA_st>
MA_dt* MovingSum<MA_dt, MA_st>::getData(void) {
  return data;
}

// ----------------------------------------------------------------------------------------------------------------------------------------
// getFilterLength - returns the Filter's Length.
// ----------------------------------------------------------------------------------------------------------------------------------------

template <class MA_dt, class MA_st>
unsigned short MovingSum<MA_dt, MA_st>::getFilterLength(void) const {
  return filter_length;
}

// ----------------------------------------------------------------------------------------------------------------------------------------
// getCurrentFilterLength - returns the current Filter's Length.
// ----------------------------------------------------------------------------------------------------------------------------------------

template <class MA_dt, class MA_st>
unsigned short MovingSum<MA_dt, MA_st>::getCurrentFilterLength(void) const {
  return filter_complete ? filter_length : (index + 1);
}

#endif

// ----------------------------------------------------------------------------------------------------------------------------------------
// END
// ----------------------------------------------------------------------------------------------------------------------------------------
