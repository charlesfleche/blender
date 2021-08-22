/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#pragma once

/** \file
 * \ingroup fn
 */

#include "FN_multi_function_procedure.hh"

namespace blender::fn {

class MFInstructionCursor {
 private:
  MFInstruction *instruction_ = nullptr;
  /* Only used when it is a branch instruction. */
  bool branch_output_ = false;
  /* Only used when instruction is null. */
  bool is_entry_ = false;

 public:
  MFInstructionCursor() = default;

  MFInstructionCursor(MFCallInstruction &instruction);
  MFInstructionCursor(MFDestructInstruction &instruction);
  MFInstructionCursor(MFBranchInstruction &instruction, bool branch_output);
  MFInstructionCursor(MFDummyInstruction &instruction);

  static MFInstructionCursor Entry();

  void insert(MFProcedure &procedure, MFInstruction *new_instruction);
};

class MFProcedureBuilder {
 private:
  MFProcedure *procedure_ = nullptr;
  Vector<MFInstructionCursor> cursors_;

 public:
  struct Branch;
  struct Loop;

  MFProcedureBuilder(MFProcedure &procedure,
                     MFInstructionCursor initial_cursor = MFInstructionCursor::Entry());

  MFProcedureBuilder(Span<MFProcedureBuilder *> builders);

  MFProcedureBuilder(Branch &branch);

  void set_cursor(const MFInstructionCursor &cursor);
  void set_cursor(Span<MFInstructionCursor> cursors);
  void set_cursor(Span<MFProcedureBuilder *> builders);
  void set_cursor_after_branch(Branch &branch);
  void set_cursor_after_loop(Loop &loop);

  void add_destruct(MFVariable &variable);
  void add_destruct(Span<MFVariable *> variables);

  Branch add_branch(MFVariable &condition);

  Loop add_loop();
  void add_loop_continue(Loop &loop);
  void add_loop_break(Loop &loop);

  MFCallInstruction &add_call_with_no_variables(const MultiFunction &fn);
  MFCallInstruction &add_call_with_all_variables(const MultiFunction &fn,
                                                 Span<MFVariable *> param_variables);

  Vector<MFVariable *> add_call(const MultiFunction &fn,
                                Span<MFVariable *> input_and_mutable_variables = {});

  template<int OutputN>
  std::array<MFVariable *, OutputN> add_call(const MultiFunction &fn,
                                             Span<MFVariable *> input_and_mutable_variables = {});

  void add_parameter(MFParamType::InterfaceType interface_type, MFVariable &variable);
  MFVariable &add_parameter(MFParamType param_type, std::string name = "");

  MFVariable &add_input_parameter(MFDataType data_type, std::string name = "");
  template<typename T> MFVariable &add_single_input_parameter(std::string name = "");
  template<typename T> MFVariable &add_single_mutable_parameter(std::string name = "");

  void add_output_parameter(MFVariable &variable);

 private:
  void link_to_cursors(MFInstruction *instruction);
};

struct MFProcedureBuilder::Branch {
  MFProcedureBuilder branch_true;
  MFProcedureBuilder branch_false;
};

struct MFProcedureBuilder::Loop {
  MFInstruction *begin = nullptr;
  MFDummyInstruction *end = nullptr;
};

/* --------------------------------------------------------------------
 * MFInstructionCursor inline methods.
 */

inline MFInstructionCursor::MFInstructionCursor(MFCallInstruction &instruction)
    : instruction_(&instruction)
{
}

inline MFInstructionCursor::MFInstructionCursor(MFDestructInstruction &instruction)
    : instruction_(&instruction)
{
}

inline MFInstructionCursor::MFInstructionCursor(MFBranchInstruction &instruction,
                                                bool branch_output)
    : instruction_(&instruction), branch_output_(branch_output)
{
}

inline MFInstructionCursor::MFInstructionCursor(MFDummyInstruction &instruction)
    : instruction_(&instruction)
{
}

inline MFInstructionCursor MFInstructionCursor::Entry()
{
  MFInstructionCursor cursor;
  cursor.is_entry_ = true;
  return cursor;
}

/* --------------------------------------------------------------------
 * MFProcedureBuilder inline methods.
 */

inline MFProcedureBuilder::MFProcedureBuilder(Branch &branch)
    : MFProcedureBuilder(*branch.branch_true.procedure_)
{
  this->set_cursor_after_branch(branch);
}

inline MFProcedureBuilder::MFProcedureBuilder(MFProcedure &procedure,
                                              MFInstructionCursor initial_cursor)
    : procedure_(&procedure), cursors_({initial_cursor})
{
}

inline MFProcedureBuilder::MFProcedureBuilder(Span<MFProcedureBuilder *> builders)
    : MFProcedureBuilder(*builders[0]->procedure_)
{
  this->set_cursor(builders);
}

inline void MFProcedureBuilder::set_cursor(const MFInstructionCursor &cursor)
{
  cursors_ = {cursor};
}

inline void MFProcedureBuilder::set_cursor(Span<MFInstructionCursor> cursors)
{
  cursors_ = cursors;
}

inline void MFProcedureBuilder::set_cursor_after_branch(Branch &branch)
{
  this->set_cursor({&branch.branch_false, &branch.branch_true});
}

inline void MFProcedureBuilder::set_cursor_after_loop(Loop &loop)
{
  this->set_cursor(MFInstructionCursor{*loop.end});
}

inline void MFProcedureBuilder::set_cursor(Span<MFProcedureBuilder *> builders)
{
  cursors_.clear();
  for (MFProcedureBuilder *builder : builders) {
    cursors_.extend(builder->cursors_);
  }
}

template<int OutputN>
inline std::array<MFVariable *, OutputN> MFProcedureBuilder::add_call(
    const MultiFunction &fn, Span<MFVariable *> input_and_mutable_variables)
{
  Vector<MFVariable *> output_variables = this->add_call(fn, input_and_mutable_variables);
  BLI_assert(output_variables.size() == OutputN);

  std::array<MFVariable *, OutputN> output_array;
  initialized_copy_n(output_variables.data(), OutputN, output_array.data());
  return output_array;
}

inline void MFProcedureBuilder::add_parameter(MFParamType::InterfaceType interface_type,
                                              MFVariable &variable)
{
  procedure_->add_parameter(interface_type, variable);
}

inline MFVariable &MFProcedureBuilder::add_parameter(MFParamType param_type, std::string name)
{
  MFVariable &variable = procedure_->new_variable(param_type.data_type(), std::move(name));
  this->add_parameter(param_type.interface_type(), variable);
  return variable;
}

inline MFVariable &MFProcedureBuilder::add_input_parameter(MFDataType data_type, std::string name)
{
  return this->add_parameter(MFParamType(MFParamType::Input, data_type), std::move(name));
}

template<typename T>
inline MFVariable &MFProcedureBuilder::add_single_input_parameter(std::string name)
{
  return this->add_parameter(MFParamType::ForSingleInput(CPPType::get<T>()), std::move(name));
}

template<typename T>
inline MFVariable &MFProcedureBuilder::add_single_mutable_parameter(std::string name)
{
  return this->add_parameter(MFParamType::ForMutableSingle(CPPType::get<T>()), std::move(name));
}

inline void MFProcedureBuilder::add_output_parameter(MFVariable &variable)
{
  this->add_parameter(MFParamType::Output, variable);
}

inline void MFProcedureBuilder::link_to_cursors(MFInstruction *instruction)
{
  for (MFInstructionCursor &cursor : cursors_) {
    cursor.insert(*procedure_, instruction);
  }
}

}  // namespace blender::fn