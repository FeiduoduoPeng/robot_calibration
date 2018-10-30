#ifndef GEOMETRY_DATATYPE_H
#define GEOMETRY_DATATYPE_H

#include <iostream>
#include <math.h>
#include <tf/tf.h>

using namespace std;

namespace robot_calibration
{
struct Point2 {
    double x;
    double y;
};

/**
 * Represents a vector (x, y) in 2-dimensional real space.
 */
template<typename T>
class Vector2
{
public:
    /**
   * Default constructor
   */
    Vector2()
    {
        m_Values[0] = 0;
        m_Values[1] = 0;
    }

    /**
   * Constructor initializing vector location
   * @param x
   * @param y
   */
    Vector2(T x, T y)
    {
        m_Values[0] = x;
        m_Values[1] = y;
    }

public:
    /**
   * Gets the x-coordinate of this vector2
   * @return the x-coordinate of the vector2
   */
    inline const T& GetX() const
    {
        return m_Values[0];
    }

    /**
   * Sets the x-coordinate of this vector2
   * @param x the x-coordinate of the vector2
   */
    inline void SetX(const T& x)
    {
        m_Values[0] = x;
    }

    /**
   * Gets the y-coordinate of this vector2
   * @return the y-coordinate of the vector2
   */
    inline const T& GetY() const
    {
        return m_Values[1];
    }

    /**
   * Sets the y-coordinate of this vector2
   * @param y the y-coordinate of the vector2
   */
    inline void SetY(const T& y)
    {
        m_Values[1] = y;
    }

    inline void SetValue(const T&x,const T& y)
    {
        m_Values[0]  = x;
        m_Values[1]  = y;
    }

    /**
   * Floor point operator
   * 取两个中的小值
   * @param rOther
   */
    inline void MakeFloor(const Vector2& rOther)
    {
        if ( rOther.m_Values[0] < m_Values[0] ) m_Values[0] = rOther.m_Values[0];
        if ( rOther.m_Values[1] < m_Values[1] ) m_Values[1] = rOther.m_Values[1];
    }

    /**
   * Ceiling point operator
   * 取两个中的大值
   * @param rOther
   */
    inline void MakeCeil(const Vector2& rOther)
    {
        if ( rOther.m_Values[0] > m_Values[0] ) m_Values[0] = rOther.m_Values[0];
        if ( rOther.m_Values[1] > m_Values[1] ) m_Values[1] = rOther.m_Values[1];
    }

    /**
   * Returns the square of the length of the vector
   * @return square of the length of the vector
   */
    inline double SquaredLength() const
    {
        return m_Values[0]*m_Values[0] + m_Values[1]*m_Values[1];
    }

    /**
   * Returns the length of the vector (x and y).
   * @return length of the vector
   */
    inline double Length() const
    {
        return sqrt(SquaredLength());
    }

    /**
   * Returns the square distance to the given vector
   * @returns square distance to the given vector
   */
    inline double SquaredDistance(const Vector2& rOther) const
    {
        return (*this - rOther).SquaredLength();
    }

    /**
   * Gets the distance to the other vector2
   * @param rOther
   * @return distance to other vector2
   */
    inline double Distance(const Vector2& rOther) const
    {
        return sqrt(SquaredDistance(rOther));
    }

public:
    /**
   * In place Vector2 addition.
   */
    inline void operator += (const Vector2& rOther)
    {
        m_Values[0] += rOther.m_Values[0];
        m_Values[1] += rOther.m_Values[1];
    }

    /**
   * In place Vector2 subtraction.
   */
    inline void operator -= (const Vector2& rOther)
    {
        m_Values[0] -= rOther.m_Values[0];
        m_Values[1] -= rOther.m_Values[1];
    }

    /**
   * Addition operator
   * @param rOther
   * @return vector resulting from adding this vector with the given vector
   */
    inline const Vector2 operator + (const Vector2& rOther) const
    {
        return Vector2(m_Values[0] + rOther.m_Values[0], m_Values[1] + rOther.m_Values[1]);
    }

    /**
   * Subtraction operator
   * @param rOther
   * @return vector resulting from subtracting this vector from the given vector
   */
    inline const Vector2 operator - (const Vector2& rOther) const
    {
        return Vector2(m_Values[0] - rOther.m_Values[0], m_Values[1] - rOther.m_Values[1]);
    }

    /**
   * In place scalar division operator
   * @param scalar
   */
    inline void operator /= (T scalar)
    {
        m_Values[0] /= scalar;
        m_Values[1] /= scalar;
    }

    /**
   * Divides a Vector2
   * @param scalar
   * @return scalar product
   */
    inline const Vector2 operator / (T scalar) const
    {
        return Vector2(m_Values[0] / scalar, m_Values[1] / scalar);
    }

    /**
   * Computes the dot product between the two vectors
   * @param rOther
   * @return dot product
   */
    inline double operator * (const Vector2& rOther) const
    {
        return m_Values[0] * rOther.m_Values[0] + m_Values[1] * rOther.m_Values[1];
    }

    /**
   * Scales the vector by the given scalar
   * @param scalar
   */
    inline const Vector2 operator * (T scalar) const
    {
        return Vector2(m_Values[0] * scalar, m_Values[1] * scalar);
    }

    /**
   * Subtract the vector by the given scalar
   * @param scalar
   */
    inline const Vector2 operator - (T scalar) const
    {
        return Vector2(m_Values[0] - scalar, m_Values[1] - scalar);
    }

    /**
   * In place scalar multiplication operator
   * @param scalar
   */
    inline void operator *= (T scalar)
    {
        m_Values[0] *= scalar;
        m_Values[1] *= scalar;
    }

    /**
   * Equality operator returns true if the corresponding x, y values of each Vector2 are the same values.
   * @param rOther
   */
    inline bool operator == (const Vector2& rOther) const
    {
        return (m_Values[0] == rOther.m_Values[0] && m_Values[1] == rOther.m_Values[1]);
    }

    /**
   * Inequality operator returns true if any of the corresponding x, y values of each Vector2 not the same.
   * @param rOther
   */
    inline bool operator != (const Vector2& rOther) const
    {
        return (m_Values[0] != rOther.m_Values[0] || m_Values[1] != rOther.m_Values[1]);
    }

    /**
   * Less than operator
   * @param rOther
   * @return true if left vector is less than right vector
   */
    inline bool operator < (const Vector2& rOther) const
    {
        if (m_Values[0] < rOther.m_Values[0])
            return true;
        else if (m_Values[0] > rOther.m_Values[0])
            return false;
        else
            return (m_Values[1] < rOther.m_Values[1]);
    }

    /**
   * Write Vector2 onto output stream
   * @param rStream output stream
   * @param rVector to write
   */
    friend inline std::ostream& operator << (std::ostream& rStream, const Vector2& rVector)
    {
        rStream << rVector.GetX() << " " << rVector.GetY();
        return rStream;
    }

    /**
   * Read Vector2 from input stream
   * @param rStream input stream
   */
    friend inline std::istream& operator >> (std::istream& rStream, const Vector2& /*rVector*/)
    {
        // Implement me!!  TODO(lucbettaieb): What the what?  Do I need to implement this?
        return rStream;
    }

private:
    T m_Values[2];
};  // Vector2<T>

/**
 * Defines a position (x, y) in 2-dimensional space and heading.
 */
class Pose2
{
public:
    /**
   * Default Constructor
   */
    Pose2()
        : m_Heading(0.0)
    {
    }

    /**
   * Constructor initializing pose parameters
   * @param rPosition position
   * @param heading heading
   **/
    Pose2(const Vector2<double>& rPosition, double heading)
        : m_Position(rPosition)
        , m_Heading(heading)
    {
    }

    /**
   * Constructor initializing pose parameters
   * @param x x-coordinate
   * @param y y-coordinate
   * @param heading heading
   **/
    Pose2(double x, double y, double heading)
        : m_Position(x, y)
        , m_Heading(heading)
    {
    }

    /**
   * Copy constructor
   */
    Pose2(const Pose2& rOther)
        : m_Position(rOther.m_Position)
        , m_Heading(rOther.m_Heading)
    {
    }

public:
    /**
   * Returns the x-coordinate
   * @return the x-coordinate of the pose
   */
    inline double GetX() const
    {
        return m_Position.GetX();
    }

    /**
   * Sets the x-coordinate
   * @param x the x-coordinate of the pose
   */
    inline void SetX(double x)
    {
        m_Position.SetX(x);
    }

    /**
   * Returns the y-coordinate
   * @return the y-coordinate of the pose
   */
    inline double GetY() const
    {
        return m_Position.GetY();
    }

    /**
   * Sets the y-coordinate
   * @param y the y-coordinate of the pose
   */
    inline void SetY(double y)
    {
        m_Position.SetY(y);
    }

    /**
   * Returns the position
   * @return the position of the pose
   */
    inline const Vector2<double>& GetPosition() const
    {
        return m_Position;
    }

    /**
   * Sets the position
   * @param rPosition of the pose
   */
    inline void SetPosition(const Vector2<double>& rPosition)
    {
        m_Position = rPosition;
    }

    /**
   * Returns the heading of the pose (in radians)
   * @return the heading of the pose
   */
    inline double GetHeading() const
    {
        return m_Heading;
    }

    /**
   * Sets the heading
   * @param heading of the pose
   */
    inline void SetHeading(double heading)
    {
        m_Heading = heading;
    }

    /**
   * Return the squared distance between two Pose2
   * @return squared distance
   */
    inline double SquaredDistance(const Pose2& rOther) const
    {
        return m_Position.SquaredDistance(rOther.m_Position);
    }

public:
    /**
   * Assignment operator
   */
    inline Pose2& operator = (const Pose2& rOther)
    {
        m_Position = rOther.m_Position;
        m_Heading = rOther.m_Heading;

        return *this;
    }

    /**
   * Equality operator
   */
    inline bool operator == (const Pose2& rOther) const
    {
        return (m_Position == rOther.m_Position && m_Heading == rOther.m_Heading);
    }

    /**
   * Inequality operator
   */
    inline bool operator != (const Pose2& rOther) const
    {
        return (m_Position != rOther.m_Position || m_Heading != rOther.m_Heading);
    }

    /**
   * In place Pose2 add.
   */
    inline void operator += (const Pose2& rOther)
    {
        m_Position += rOther.m_Position;
        m_Heading = tfNormalizeAngle(m_Heading + rOther.m_Heading);
    }

    /**
   * Binary Pose2 add
   * @param rOther
   * @return Pose2 sum
   */
    inline Pose2 operator + (const Pose2& rOther) const
    {
        return Pose2(m_Position + rOther.m_Position, tfNormalizeAngle(m_Heading + rOther.m_Heading));
    }

    /**
   * Binary Pose2 subtract
   * @param rOther
   * @return Pose2 difference
   */
    inline Pose2 operator - (const Pose2& rOther) const
    {
        return Pose2(m_Position - rOther.m_Position, tfNormalizeAngle(m_Heading - rOther.m_Heading));
    }

    /**
   * Read pose from input stream
   * @param rStream input stream
   */
    friend inline std::istream& operator >> (std::istream& rStream, const Pose2& /*rPose*/)
    {
        // Implement me!!
        return rStream;
    }

    /**
   * Write this pose onto output stream
   * @param rStream output stream
   * @param rPose to read
   */
    friend inline std::ostream& operator << (std::ostream& rStream, const Pose2& rPose)
    {
        rStream << rPose.m_Position.GetX() << " " << rPose.m_Position.GetY() << " " << rPose.m_Heading;
        return rStream;
    }

private:
    Vector2<double> m_Position;

    double m_Heading;
};  // Pose2
}

#endif // GEOMETRY_DATATYPE_H
