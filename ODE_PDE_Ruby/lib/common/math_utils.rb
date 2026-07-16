# frozen_string_literal: true

# ------------------------------------------------------------
# Common math utilities used across examples.
# ------------------------------------------------------------
module MathUtils
  module_function

  # Clamp a value x to the inclusive range [lo, hi].
  def clamp(x, lo, hi)
    return lo if x < lo
    return hi if x > hi
    x
  end

  # Wrap an angle (radians) into [-pi, pi] to avoid unbounded numeric drift.
  def wrap_to_pi(a)
    pi = Math::PI
    two_pi = 2.0 * pi

    while a > pi
      a -= two_pi
    end
    while a < -pi
      a += two_pi
    end
    a
  end

  # Boundary-layer saturation for Sliding Mode Control:
  # sat(z) in [-1, 1].
  def sat(z)
    clamp(z, -1.0, 1.0)
  end
end
