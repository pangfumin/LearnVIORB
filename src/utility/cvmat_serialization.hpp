#ifndef SM_OPENCV_SERIALIZATION_HPP
#define SM_OPENCV_SERIALIZATION_HPP


#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <boost/serialization/binary_object.hpp>
#include <boost/serialization/split_free.hpp>
#include <opencv2/features2d/features2d.hpp>


BOOST_SERIALIZATION_SPLIT_FREE(::cv::Mat)

namespace boost {
    namespace serialization {

/*** CV KeyFrame ***/
        template<class Archive>
        void serialize(Archive & ar, cv::KeyPoint & kf, const unsigned int version)
        {
            ar & kf.angle;
            ar & kf.class_id;
            ar & kf.octave;
            ar & kf.response;
            ar & kf.response;
            ar & kf.pt.x;
            ar & kf.pt.y;

        }



/*** Mat ***/
/** Serialization support for cv::Mat */
        template<class Archive>
        void save(Archive & ar, const ::cv::Mat& m, const unsigned int version)
        {
            size_t elem_size = m.elemSize();
            size_t elem_type = m.type();

            ar & m.cols;
            ar & m.rows;
            ar & elem_size;
            ar & elem_type;

            const size_t data_size = m.cols * m.rows * elem_size;

            ar & boost::serialization::make_array(m.ptr(), data_size);
        }

/** Serialization support for cv::Mat */
        template<class Archive>
        void load(Archive & ar, ::cv::Mat& m, const unsigned int version)
        {
            int cols, rows;
            size_t elem_size, elem_type;

            ar & cols;
            ar & rows;
            ar & elem_size;
            ar & elem_type;

            m.create(rows, cols, elem_type);
            size_t data_size = m.cols * m.rows * elem_size;

            ar & boost::serialization::make_array(m.ptr(), data_size);
        }

    }
}


#endif /* SM_OPENCV_SERIALIZATION_HPP */
