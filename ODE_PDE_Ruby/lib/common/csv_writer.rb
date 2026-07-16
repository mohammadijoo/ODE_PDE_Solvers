# frozen_string_literal: true

require "csv"
require "fileutils"

# ------------------------------------------------------------
# Minimal CSV writer utilities.
# ------------------------------------------------------------
module CsvWriter
  module_function

  # Write a CSV file given a header array and an array of columns.
  #
  # - header: ["t", "x", "theta", ...]
  # - cols:   [col0_array, col1_array, ...] with equal lengths
  def write_csv(filename, header, cols)
    raise "CSV: no columns provided." if cols.nil? || cols.empty?

    n = cols[0].length
    cols.each do |c|
      raise "CSV: column size mismatch." if c.length != n
    end

    FileUtils.mkdir_p(File.dirname(filename))

    CSV.open(filename, "w", write_headers: true, headers: header) do |csv|
      (0...n).each do |r|
        row = cols.map { |c| c[r] }
        csv << row
      end
    end
  end

  # Convenience: two-column CSV writer.
  def write_two_columns(filename, h1, h2, c1, c2)
    raise "CSV: column size mismatch." if c1.length != c2.length
    write_csv(filename, [h1, h2], [c1, c2])
  end
end
