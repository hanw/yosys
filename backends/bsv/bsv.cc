/*
 *  Copyright (C) 2020  Han Wang <hw342@cornell.edu>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include "kernel/rtlil.h"
#include "kernel/register.h"
#include "kernel/sigtools.h"
#include "kernel/celltypes.h"
#include "kernel/cellaigs.h"
#include "kernel/log.h"
#include <string>
#include <vector>
#include <cctype>

USING_YOSYS_NAMESPACE
PRIVATE_NAMESPACE_BEGIN

struct BsvWriter
{
	std::ostream &f;
    bool use_selection;
    const std::vector<std::string>& clocks;
    const std::vector<std::string>& resets;
    const std::vector<std::string>& params;
    const std::vector<std::string>& groups;
    const std::string& interface;

	Design *design;
	Module *module;

	SigMap sigmap;
	int sigidcounter;
	dict<SigBit, string> sigids;

    std::map<std::string, std::vector<std::string>> interface_groups;

	BsvWriter(std::ostream &f, bool use_selection, const std::vector<std::string>& clocks,
            const std::vector<std::string>& resets, const std::vector<std::string>& params,
            const std::vector<std::string>& groups,
            const std::string& interface) :
        f(f), use_selection(use_selection), clocks(clocks), resets(resets),
        params(params), groups(groups),
        interface(interface) { }

    string to_type_name(string str) {
        string newstr;
        newstr = str;
        newstr[0] = toupper(newstr[0]);
        return newstr;
    }

	string get_string(string str)
	{
		string newstr;
		for (char c : str) {
			if (c == '\\')
				newstr += c;
			newstr += c;
		}
		return newstr;
	}

	string get_name(IdString name)
	{
		return get_string(RTLIL::unescape_id(name));
	}

	string get_bits(SigSpec sig)
	{
		bool first = true;
		string str = "[";
		for (auto bit : sigmap(sig)) {
			str += first ? " " : ", ";
			first = false;
			if (sigids.count(bit) == 0) {
				string &s = sigids[bit];
				if (bit.wire == nullptr) {
					if (bit == State::S0) s = "\"0\"";
					else if (bit == State::S1) s = "\"1\"";
					else if (bit == State::Sz) s = "\"z\"";
					else s = "\"x\"";
				} else
					s = stringf("%d", sigidcounter++);
			}
			str += sigids[bit];
		}
		return str + " ]";
	}

    bool is_clock_or_reset(std::string s) {
        if (std::find(clocks.begin(), clocks.end(), s) != clocks.end())
            return true;
        if (std::find(resets.begin(), resets.end(), s) != resets.end())
            return true;
        return false;
    }

    void regroup_ports(std::vector<std::string> groups, std::vector<std::string> ports) {
        if (groups.empty()) return;
        std::sort(groups.begin(), groups.end());
        for (auto p : ports) {
            for (auto g : groups) {
                if (p.find(g) != std::string::npos) {
                    interface_groups[g].push_back(p);
                }
            }
        }
    }

    void generate_interface(Module *module, std::string interface) {
        std::vector<std::string> ports;
        std::map<std::string, std::string> port_directions;
        std::map<std::string, size_t> port_sizes;
        std::map<std::string, RTLIL::IdString> port_map;
		for (auto n : module->ports) {
			Wire *w = module->wire(n);
            if (is_clock_or_reset(get_name(n)))
                continue;
            auto port_name = get_name(n);
            ports.push_back(port_name);
			auto direction = w->port_input ? w->port_output ? "inout" : "input" : "output";
            port_directions.emplace(port_name, direction);
			port_sizes.emplace(port_name, sigmap(w).size());
            port_map.emplace(port_name, n);
        }
        regroup_ports(groups, ports);

        for (auto ig : interface_groups) {
            f << stringf("(* always_ready, always_enabled *)\n");
            f << stringf("interface %s;\n", to_type_name(ig.first).c_str());
            for (auto p : ig.second) {
                IdString id = port_map.at(p);
                Wire *w = module->wire(id);
                auto size = port_sizes.at(p);
                std::string sig = p.substr(ig.first.length()+1);
                std::string direction = w->port_input ? w->port_output ? "inout" : "input" : "output";
                if (direction.compare("input") == 0)
                    f << stringf("   method Action %s(Bit#(%lu) v);\n", sig.c_str(), size);
                else if (direction.compare("output") == 0)
                    f << stringf("   method Bit#(%lu) %s();\n", size, sig.c_str());
            }
            f << stringf("endinterface\n");
        }

        f << stringf("(* always_ready, always_enabled *)\n");
        f << stringf("interface %s;\n", to_type_name(interface).c_str());
        for (auto ig : interface_groups) {
            f << stringf("    interface %s %s;\n", to_type_name(ig.first).c_str(), ig.first.c_str());
        }
        f << stringf("endinterface\n");
    }

    void generate_instance(Module *module, std::string interface) {
        // map string to IR node
        std::map<std::string, RTLIL::IdString> portmap;
        for (auto p : module->ports)
            portmap.emplace(get_name(p), p);

        auto signals = interface_groups[interface];
        f << stringf("    interface %s %s;\n", to_type_name(interface).c_str(), interface.c_str());
        for (auto s : signals) {
            IdString id = portmap.at(s);
			Wire *w = module->wire(id);
            std::string direction = w->port_input ? w->port_output ? "inout" : "input" : "output";
            std::string sig = s.substr(interface.length()+1);
            const char* name = sig.c_str();
            if (direction.compare("input") == 0)
                f << stringf("        method %s(%s) enable((*in_high*) EN_%s);\n",
                        name, name, name);
            else if (direction.compare("output") == 0)
                f << stringf("        method %s %s();\n", name, name);
            else if (direction.compare("inout") == 0)
                f << stringf("        inout %s;\n", name);
        }
        f << stringf("    endinterface\n");
    }

    void generate_schedule(Module *module) {
        std::vector<std::string> ports;
		for (auto n : module->ports) {
            if (is_clock_or_reset(get_name(n)))
                continue;
            ports.push_back(get_name(n));
        }
        f << stringf("    schedule(\n");
        for (auto it = ports.begin(); it != ports.end(); ++it) {
            f << stringf("        %s", it->c_str());
            if (it != --ports.end())
                f << stringf(",\n");
        }
        f << stringf("\n    ) CF (\n");
        for (auto it = ports.begin(); it != ports.end(); ++it) {
            f << stringf("        %s", it->c_str());
            if (it != --ports.end())
                f << stringf(",\n");
        }
        f << stringf("\n    );\n");
    }

	void write_module(Module *module_)
	{
		module = module_;
		log_assert(module->design == design);
		sigmap.set(module);
		sigids.clear();

		// reserve 0 and 1 to avoid confusion with "0" and "1"
		sigidcounter = 2;

        generate_interface(module, interface.c_str());

        f << stringf("import \"BVI\" %s=\n", get_name(module->name).c_str());
        f << stringf("module mk%s#(", interface.c_str());

        for (auto c : clocks)
            f << stringf("Clock %s, ", c.c_str());
        for (auto it = resets.begin(); it != resets.end(); ++it) {
            f << stringf("Reset %s", it->c_str());
            if (it != --resets.end())
                f << stringf(", ");
        }
        f << stringf(")");
        f << stringf("(%s);\n", interface.c_str());

        for (auto c : clocks)
            f << stringf("    input_clock %s() = %s;\n", c.c_str(), c.c_str());
        for (auto r : resets)
            f << stringf("    input_reset %s() = %s;\n", r.c_str(), r.c_str());

        for (auto ig : interface_groups) {
            generate_instance(module, ig.first.c_str());
        }

        generate_schedule(module);

        f << stringf("endmodule\n");
#if 0
        for (auto n : module->ports) {
			Wire *w = module->wire(n);
			if (use_selection && !module->selected(w))
				continue;
			f << stringf("%s\n", get_name(n).c_str());
			f << stringf("direction: %s\n", w->port_input ? w->port_output ? "inout" : "input" : "output");
			f << stringf("bits: %s\n", get_bits(w).c_str());
			first = false;
		}
#endif
    }

	void write_design(Design *design_)
	{
		design = design_;
		design->sort();

		vector<Module*> modules = use_selection ? design->selected_modules() : design->modules();
		bool first_module = true;
		for (auto mod : modules) {
			if (!first_module)
				f << stringf(",\n");
			write_module(mod);
			first_module = false;
		}
	}
};

struct BsvBackend: public Backend {
    BsvBackend() : Backend("bsv", "write a wrapper to embed Verilog or VHDL module in a BSV design") { }
    void help() YS_OVERRIDE
    {
		log("\n");
		log("    write_bsv [options] [filename]\n");
        log("    -c <clock>\n");
        log("        clock name");
        log("    -r <reset>\n");
        log("        reset name");
        log("    -p <param>\n");
        log("        parameter name");
        log("    -i <interface>\n");
        log("        module and module interface name");
        log("    -g <prefix>\n");
        log("        group signals with 'prefix' to the same interface");
		log("\n");
    }
	void execute(std::ostream *&f, std::string filename, std::vector<std::string> args, RTLIL::Design *design) YS_OVERRIDE
	{
        std::vector<std::string> clocks;
        std::vector<std::string> resets;
        std::vector<std::string> params;
        std::vector<std::string> groups;
        std::string interface;

		size_t argidx;
		for (argidx = 1; argidx < args.size(); argidx++)
		{
            if (args[argidx] == "-c" && argidx+1 < args.size()) {
                clocks.push_back(args[++argidx]);
                continue;
            }
            if (args[argidx] == "-r" && argidx+1 < args.size()) {
                resets.push_back(args[++argidx]);
                continue;
            }
            if (args[argidx] == "-p" && argidx+1 < args.size()) {
                params.push_back(args[++argidx]);
                continue;
            }
            if (args[argidx] == "-i" && argidx+1 < args.size()) {
                interface = args[++argidx];
                continue;
            }
            if (args[argidx] == "-g" && argidx+1 < args.size()) {
                groups.push_back(args[++argidx]);
                continue;
            }
			break;
		}

		extra_args(f, filename, args, argidx);
		log_header(design, "Executing JSON backend.\n");

		BsvWriter bsv_writer(*f, false, clocks, resets, params, groups, interface);
		bsv_writer.write_design(design);
	}
} BsvBackend;

struct BsvPass : public Pass {
	BsvPass() : Pass("bsv", "write a wrapper to embed Verilog or VHDL module in a BSV design") { }
	void help() YS_OVERRIDE
	{
		log("\n");
		log("    bsv [options] [selection]\n");
		log("\n");
		log("    -o <filename>\n");
		log("        write to the specified file.\n");
        log("    -c <clock>\n");
        log("        clock name");
        log("    -r <reset>\n");
        log("        reset name");
        log("    -p <param>\n");
        log("        parameter name");
        log("    -i <interface>\n");
        log("        module and module interface name");
        log("    -g <prefix>\n");
        log("        group signals with 'prefix' to the same interface");
		log("\n");
	}
	void execute(std::vector<std::string> args, RTLIL::Design *design) YS_OVERRIDE
	{
		std::string filename;
        std::vector<std::string> clocks;
        std::vector<std::string> resets;
        std::vector<std::string> params;
        std::vector<std::string> groups;
        std::string interface;

		size_t argidx;
		for (argidx = 1; argidx < args.size(); argidx++)
		{
			if (args[argidx] == "-o" && argidx+1 < args.size()) {
				filename = args[++argidx];
				continue;
			}
            if (args[argidx] == "-c" && argidx+1 < args.size()) {
                clocks.push_back(args[++argidx]);
                continue;
            }
            if (args[argidx] == "-r" && argidx+1 < args.size()) {
                resets.push_back(args[++argidx]);
                continue;
            }
            if (args[argidx] == "-p" && argidx+1 < args.size()) {
                params.push_back(args[++argidx]);
                continue;
            }
            if (args[argidx] == "-i" && argidx+1 < args.size()) {
                interface = args[++argidx];
                continue;
            }
            if (args[argidx] == "-g" && argidx+1 < args.size()) {
                groups.push_back(args[++argidx]);
                continue;
            }
			break;
		}
		extra_args(args, argidx, design);

		std::ostream *f;
		std::stringstream buf;

		if (!filename.empty()) {
			rewrite_filename(filename);
			std::ofstream *ff = new std::ofstream;
			ff->open(filename.c_str(), std::ofstream::trunc);
			if (ff->fail()) {
				delete ff;
				log_error("Can't open file `%s' for writing: %s\n", filename.c_str(), strerror(errno));
			}
			f = ff;
		} else {
			f = &buf;
		}

		BsvWriter bsv_writer(*f, true, clocks, resets, params, groups, interface);
		bsv_writer.write_design(design);
		if (!filename.empty()) {
			delete f;
		} else {
			log("%s", buf.str().c_str());
		}
	}
} BsvPass;

PRIVATE_NAMESPACE_END
