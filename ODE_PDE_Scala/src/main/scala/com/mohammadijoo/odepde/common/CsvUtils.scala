package com.mohammadijoo.odepde.common

import java.nio.charset.StandardCharsets
import java.nio.file.{Files, Path}
import scala.jdk.CollectionConverters._

/**
  * Minimal CSV utilities.
  *
  * The goal is to keep the numerical examples self-contained and reproducible:
  * - no external CSV libraries
  * - clear headers
  * - strict column-length checks
  */
object CsvUtils {

  /**
    * Writes a 2-column CSV with a header row.
    *
    * @throws IllegalArgumentException if column lengths do not match
    */
  def writeTwoColumns(
      file: Path,
      h1: String,
      h2: String,
      c1: Seq[Double],
      c2: Seq[Double]
  ): Unit = {
    if (c1.length != c2.length) {
      throw new IllegalArgumentException("CSV write error: column sizes do not match.")
    }

    val lines =
      (Seq(s"$h1,$h2") ++ c1.indices.map(i => s"${c1(i)},${c2(i)}")).asJava

    Files.createDirectories(file.getParent)
    Files.write(file, lines, StandardCharsets.UTF_8)
  }

  /**
    * Writes an N-column CSV with a header row.
    *
    * @param header column names (size = N)
    * @param cols   column vectors (size = N, and all columns same length)
    */
  def writeColumns(
      file: Path,
      header: Seq[String],
      cols: Seq[Seq[Double]]
  ): Unit = {
    if (cols.isEmpty) throw new IllegalArgumentException("CSV write error: no columns.")
    val n = cols.head.length
    if (!cols.forall(_.length == n)) throw new IllegalArgumentException("CSV write error: column size mismatch.")
    if (header.length != cols.length) throw new IllegalArgumentException("CSV write error: header/column count mismatch.")

    val headerLine = header.mkString(",")

    val lines =
      (Seq(headerLine) ++ (0 until n).map { r =>
        cols.indices.map(c => cols(c)(r)).mkString(",")
      }).asJava

    Files.createDirectories(file.getParent)
    Files.write(file, lines, StandardCharsets.UTF_8)
  }
}
