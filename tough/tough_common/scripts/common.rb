#!/usr/bin/env ruby

module Common
  # The Vector class represents the generic vector containing elements.
  # Since it's commonly used to keep coordinate system
  # related information, its elements are labeled by x, y, z.
  class Vector
    def initialize
      @x = 0
      @y = 0
      @z = 0
    end

    def set(x, y, z)
      @x = x
      @y = y
      @z = z
    end

    def distance(pt)
      Math.sqrt(Math.pow(@x - pt.x, 2) +
                Math.pow(@y - pt.y, 2) +
                Math.pow(@z - pt.z, 2))
    end

    attr_accessor :x
    attr_accessor :y
    attr_accessor :z
  end

  # A quaternion class.
  class Quaternion
    def initialize
      @x = 0
      @y = 0
      @z = 0
      @w = 1
    end

    def set(roll, pitch, yaw)
      phi = roll / 2.0
      the = pitch / 2.0
      psi = yaw / 2.0

      @w = Math.cos(phi) * Math.cos(the) * Math.cos(psi) +
           Math.sin(phi) * Math.sin(the) * Math.sin(psi)
      @x = Math.sin(phi) * Math.cos(the) * Math.cos(psi) -
           Math.cos(phi) * Math.sin(the) * Math.sin(psi)
      @y = Math.cos(phi) * Math.sin(the) * Math.cos(psi) +
           Math.sin(phi) * Math.cos(the) * Math.sin(psi)
      @z = Math.cos(phi) * Math.cos(the) * Math.sin(psi) -
           Math.sin(phi) * Math.sin(the) * Math.cos(psi)
      normalize
    end

    def *(other)
      result = Quaternion.new
      result.w = @w * other.w - @x * other.x - @y * other.y - @z * other.z
      result.x = @w * other.x + @x * other.w + @y * other.z - @z * other.y
      result.y = @w * other.y - @x * other.z + @y * other.w + @z * other.x
      result.z = @w * other.z + @x * other.y - @y * other.x + @z * other.w
      return result
    end

    def inverse
      q = Quaternion.new
      q.w = @w
      q.x = @x
      q.y = @y
      q.z = @z

      s = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z

      if s.zero?
        q.w = 1.0
        q.x = 0.0
        q.y = 0.0
        q.z = 0.0
      else
        q.w =  q.w / s
        q.x = -q.x / s
        q.y = -q.y / s
        q.z = -q.z / s
      end
    end

    def normalize
      s = Math.sqrt(@w * @w + @x * @x + @y * @y + @z * @z)

      if s.zero?
        @w = 1.0
        @x = 0.0
        @y = 0.0
        @z = 0.0
      else
        @w /= s
        @x /= s
        @y /= s
        @z /= s
      end
    end

    attr_accessor :x
    attr_accessor :y
    attr_accessor :z
    attr_accessor :w
  end

  # \brief Encapsulates a position and rotation in three space.
  class Pose
    def initialize
      @p = Vector.new
      @q = Quaternion.new
    end

    def set(x, y, z, roll, pitch, yaw)
      @p.set(x, y, z)
      @q.set(roll, pitch, yaw)
    end

    def +(other)
      result = Pose.new

      tmp = Quaternion.new
      tmp.w = 0.0
      tmp.x = other.p.x
      tmp.y = other.p.y
      tmp.z = other.p.z
      tmp = @q * (tmp * @q.inverse)

      result.p.set(@p.x + tmp.x, @p.y + tmp.y, @p.z + tmp.z)
      result.q = other.q * @q

      return result
    end

    def distance(other)
      @p.distance(other.p)
    end

    attr_accessor :p
    attr_accessor :q
  end

  # A Time class, can be used to hold wall- or sim-time
  # stored as sec and nano-sec.
  class Time
    def initialize
      @sec = 0
      @nsec = 0
    end

    def eql?(other)
      @sec == other.sec && @nsec == other.nsec
    end

    def hash
      @sec.to_f + @nsec.to_f * 1e-9
    end

    def -(other)
      result = Time.new
      result.sec = @sec - other.sec
      result.nsec = @nsec - other.nsec
      result.correct
      return result
    end

    def >=(other)
      if (@sec.to_i < other.sec.to_i)
        return false
      end

      if (@sec.to_i > other.sec.to_i)
        return true
      end

      return @nsec.to_i >= other.nsec.to_i
    end

    def correct
      if @sec > 0 && @nsec < 0
        n = (@nsec / 1_000_000_000).abs.to_i + 1
        @sec -= n
        @nsec += n * 1_000_000_000
      end
      if @sec < 0 && @nsec > 0
        n = (@nsec / 1_000_000_000).abs.to_i + 1
        @sec += n
        @nsec -= n * 1_000_000_000
      end

      @sec += (@nsec / 1_000_000_000).to_i
      @nsec = (@nsec % 1_000_000_000).to_i
    end

    attr_accessor :sec
    attr_accessor :nsec
  end

  class Color
    def initialize
      @r = 0.0
      @g = 0.0
      @b = 0.0
      @a = 1.0
    end

    def ==(other)
      return @r == other.r && @g == other.g && @b == other.b && @a == other.a
    end

    def difference(other)
      return Math.sqrt((@r-other.r) * (@r-other.r) +
                       (@g-other.g) * (@g-other.g) +
                       (@b-other.b) * (@b-other.b) +
                       (@a-other.a) * (@a-other.a))
    end

    attr_accessor :r
    attr_accessor :g
    attr_accessor :b
    attr_accessor :a
  end
end
