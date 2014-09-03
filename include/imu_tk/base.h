#pragma once

#include <vector>
#include <iostream>
#include <Eigen/Core>

namespace imu_tk
{

struct DataInterval
{
public:
  DataInterval() {};
  DataInterval ( int idx0, int idx1, double ts0, double ts1 ) :
    start_idx ( idx0 ), end_idx ( idx1 ),start_ts ( ts0 ), end_ts ( ts1 ) {};
  DataInterval ( int idx0, int idx1 ) :
    start_idx ( idx0 ), end_idx ( idx1 ),start_ts ( -1 ), end_ts ( -1 ) {};
  int start_idx, end_idx;
  double start_ts, end_ts;
};
    
//     template< typename NewTypeT >
//     ParamVector( const ParamVector< NewTypeT > &rhs )
//     {
//         // convert
//     }
// 
//     template < typename NewTypeT >
//     ParamVector& operator=( const ParamVector< NewTypeT > &rhs )
//     {
//         // do some conversion thigns
//         return *this;
//     }
//     
template <typename _T = double> class TriadData
{
public:
  TriadData() {};
   
  TriadData ( _T timestamp, _T x, _T y, _T z ) :
    timestamp_ ( timestamp ),
    data_ ( x, y, z ) {};

  TriadData ( _T timestamp, const Eigen::Matrix< _T, 3, 1> &data ) :
    timestamp_ ( timestamp ),
    data_ ( data ) {};

  TriadData ( _T timestamp, const _T *data ) :
    timestamp_ ( timestamp ),
    data_ ( data[0], data[1], data[2] ) {};

  //Copy constructor
  TriadData( const TriadData &o ) :
   timestamp_(o.timestamp_), data_(o.data_) {};
  
  // Copy assignment operator
  TriadData & operator = (const TriadData &o )
  {
    timestamp_ = o.timestamp_;
    data_ = o.data_;
    return *this;
  };
 
  // Supporting coercion using member template constructor.
  // This is not a copy constructor, but behaves similarly.
  template< typename _newT >
    TriadData( const TriadData<_newT> &o )
  { 
    timestamp_ = _T(o.timestamp_);
    data_ = o.data_.template cast<_T>();
  };
  
  // Supporting coercion using member template assignment operator.
  // This is not the copy assignment operator, but works similarly.
  template< typename _newT >
    TriadData & operator = (const TriadData<_newT> &o )
  {
    timestamp_ = _T(o.timestamp_);
    data_ = o.data_;
    return *this;
  };
    
  ~TriadData() {};

  inline const _T& timestamp() const
  {
    return timestamp_;
  };
  inline const Eigen::Matrix< _T, 3, 1>& data() const
  {
    return data_;
  };
  inline const _T& operator() ( int index ) const
  {
    return data_[index];
  };
  inline const _T& x() const
  {
    return data_[0];
  };
  inline const _T& y() const
  {
    return data_[1];
  };
  inline const _T& z() const
  {
    return data_[2];
  };

private:
  Eigen::Matrix< _T, 3, 1> data_;
  _T timestamp_;
};

template <typename _T> 
  std::ostream& operator<<(std::ostream& os, const TriadData<_T>& triad_data);


template <typename _T> 
  DataInterval checkInterval( const std::vector< TriadData<_T> > &samples, 
                              const DataInterval &interval );
template <typename _T> 
  Eigen::Matrix< _T, 3, 1> dataMean ( const std::vector< TriadData<_T> > &samples, 
                                      const DataInterval &interval = DataInterval ( -1, -1 ) );
template <typename _T>
  Eigen::Matrix< _T, 3, 1> dataVariance ( const std::vector< TriadData<_T> > &samples, 
                                          const DataInterval &interval = DataInterval ( -1, -1 ) );

/**
  * @brief If the flag only_means is set to false, for each interval 
  *        (input vector intervals) extract from the input signal 
  *        (samples) the first interval_n_samps samples, and store them 
  *        in the vector output vector extracted_samples. If the flag only_means
  *        is set to true, extract for each interval only the local mean, computed 
  *        in a interval of size interval_n_samps samples.
  *        Only intervals with at least interval_n_samps samples are considered.
  * 
  * @param samples Input 3D signal
  * @param intervals Input intervals vector
  * @param extracted_samples Output signal that contains the extracted samples
  * @param interval_n_samps Number of samples to be stracted from each interval (or 
  *                         interval size to be used to compute the local mean if
  *                         only_means is set to true)
  * @param only_means If true, extract for each interval only the local mean, computed 
  *                   in a interval of size interval_n_samps samples. The timestamp is
  *                   the one of the center of the interval.
  * 
  * @return The number of valid intervals, i.e. intervals with at least interval_n_samps.
  */
template <typename _T> 
  int extractIntervalsSamples ( const std::vector< TriadData<_T> > &samples,
                                const std::vector< DataInterval > &intervals,
                                std::vector< TriadData<_T> > &extracted_samples,
                                int interval_n_samps = 100, bool only_means = false );

/* Implementations */

template <typename _T>
  std::ostream& operator<<(std::ostream& os, const TriadData<_T>& triad_data)
{
  os<<"ts : "<<triad_data.timestamp();
  os<<" data : [ ";
  os<<triad_data.x()<<", ";
  os<<triad_data.y()<<", ";
  os<<triad_data.z()<<" ]";
  
  return os;
}

template <typename _T>
  DataInterval checkInterval( const std::vector< TriadData<_T> > &samples, 
                              const DataInterval &interval )
{
  int start_idx = interval.start_idx, end_idx = interval.end_idx;
  if( start_idx < 0) start_idx = 0;
  if( end_idx < start_idx ) end_idx = samples.size() - 1;
  
  return DataInterval( start_idx, end_idx, 
                       samples[start_idx].timestamp(), samples[end_idx].timestamp() );
}

template <typename _T>
  Eigen::Matrix< _T, 3, 1> dataMean( const std::vector< TriadData<_T> >& samples, 
                              const DataInterval& interval )
{
  DataInterval rev_interval =  checkInterval( samples, interval );
  int n_samp = rev_interval.end_idx - rev_interval.start_idx + 1;
  Eigen::Matrix< _T, 3, 1> mean(0, 0, 0);
  
  for( int i = rev_interval.start_idx; i <= rev_interval.end_idx; i++)
    mean += samples[i].data();
  
  mean /= _T(n_samp);
  
  return mean;
}

template <typename _T>
  Eigen::Matrix< _T, 3, 1> dataVariance( const std::vector< TriadData<_T> >& samples, 
                                  const DataInterval& interval )
{
  DataInterval rev_interval =  checkInterval( samples, interval );
  int n_samp = rev_interval.end_idx - rev_interval.start_idx + 1;
  Eigen::Matrix< _T, 3, 1> mean = dataMean( samples, rev_interval );
  
  Eigen::Matrix< _T, 3, 1> variance(0, 0, 0);
  for( int i = rev_interval.start_idx; i <= rev_interval.end_idx; i++)
  {
    Eigen::Matrix< _T, 3, 1> diff = samples[i].data() - mean ;
    variance += (diff.array() * diff.array()).matrix();
  }
  variance /= _T(n_samp - 1);
  
  return variance;
}

template <typename _T>
  int extractIntervalsSamples ( const std::vector< TriadData<_T> >& samples, 
                                const std::vector< DataInterval >& intervals, 
                                std::vector< TriadData<_T> >& extracted_samples, 
                                int interval_n_samps, bool only_means )
{
  // Check for valid intervals  (i.e., intervals with at least interval_n_samps samples)
  int n_valid_intervals = 0, n_static_samples;
  for( int i = 0; i < intervals.size(); i++)
  {
    if( (intervals[i].end_idx - intervals[i].start_idx) >= interval_n_samps )
      n_valid_intervals++;
  }
  
  if( only_means )
    n_static_samples = n_valid_intervals;
  else
    n_static_samples = n_valid_intervals*interval_n_samps;
      
  extracted_samples.resize(n_static_samples);
  int s_index = 0;
  // For each valid interval, extract the first interval_n_samps samples
  for( int i = 0; i < intervals.size(); i++)
  {
    if( (intervals[i].end_idx - intervals[i].start_idx) >= interval_n_samps )
    {
      if( only_means )
      {
        DataInterval mean_inerval( intervals[i].start_idx, intervals[i].start_idx + interval_n_samps -1 );
        // Take the timestamp centered in the interval where the mean is computed
        _T timestamp = samples[ intervals[i].start_idx + interval_n_samps/2 -1 ].timestamp();
        Eigen::Matrix< _T, 3, 1> mean_val = dataMean ( samples, mean_inerval );
        extracted_samples[s_index++] = TriadData<_T>(timestamp, mean_val );
      }
      else
      {
        for(int j = intervals[i].start_idx; j < intervals[i].start_idx + interval_n_samps; j++)
          extracted_samples[s_index++] = samples[j];
      }
    }
  }
  return n_valid_intervals;
}

}