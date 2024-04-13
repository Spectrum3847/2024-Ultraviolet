// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.spectrumLib.util;

/**
 * Represents a trio of three objects.
 *
 * @param <A> The first object's type.
 * @param <B> The second object's type.
 * @param <C> The second object's type.
 */
public class Trio<A, B, C> {
    private final A m_first;
    private final B m_second;
    private final C m_third;

    /**
     * Constructs a pair.
     *
     * @param first The first object.
     * @param second The second object.
     * @param third The third object
     */
    public Trio(A first, B second, C third) {
        m_first = first;
        m_second = second;
        m_third = third;
    }

    /**
     * Returns the first object.
     *
     * @return The first object.
     */
    public A getFirst() {
        return m_first;
    }

    /**
     * Returns the second object.
     *
     * @return The second object.
     */
    public B getSecond() {
        return m_second;
    }

    public C getThird() {
        return m_third;
    }

    /**
     * Returns a trio comprised of the three given objects.
     *
     * @param <A> The first object's type.
     * @param <B> The second object's type.
     * @param <C> The third object's type.
     * @param a The first object.
     * @param b The second object.
     * @param b The second object.
     * @return A trio comprised of the three given objects.
     */
    public static <A, B, C> Trio<A, B, C> of(A a, B b, C c) {
        return new Trio<A, B, C>(a, b, c);
    }
}
