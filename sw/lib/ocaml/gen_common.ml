(*
 * generic tools for modules
 *
 * Copyright (C) 2010 Gautier Hattenberger
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 *)

open Printf

type module_conf = {
    name: string;
    xml: Xml.xml;
    file: string;
    filename: string;
    vpath: string option;(* this field should be removed after transition phase *)
    param: Xml.xml list;
    targets: string list
  }

let (//) = Filename.concat

let paparazzi_conf = Env.paparazzi_home // "conf"
let modules_dir = paparazzi_conf // "modules"
let autopilot_dir = paparazzi_conf // "autopilot"

(** remove all duplicated elements of a list *)
let singletonize = fun ?(compare = Pervasives.compare) l ->
  let rec loop = fun l ->
    match l with
    | [] | [_] -> l
    | x::((x'::_) as xs) -> if compare x  x' = 0 then loop xs else x::loop xs in
  loop (List.sort Pervasives.compare l)

(** union of two lists *)
let union = fun l1 l2 -> singletonize (l1 @ l2)

(** union of a list of list *)
let union_of_lists = fun l -> singletonize (List.flatten l)

(** [targets_of_field]
    * Returns the targets of a makefile node in modules
    * Default "ap|sim" *)
let targets_of_field =
  let pipe = Str.regexp "|" in
  fun field default ->
    Str.split pipe (ExtXml.attrib_or_default field "target" default)

(** [get_autopilot_of_airframe xml]
    * Returns (autopilot xml, main freq) from airframe xml file *)
let get_autopilot_of_airframe = fun xml ->
  (* extract all "autopilot" sections *)
  let section = List.filter (fun s -> compare (Xml.tag s) "autopilot" = 0) (Xml.children xml) in
  (* Raise error if more than one modules section *)
  match section with
      [autopilot] ->
        let freq = try Some (Xml.attrib autopilot "freq") with _ -> None in
        let ap = try Xml.attrib autopilot "name" with _ -> raise Not_found in
        (autopilot_dir // ap, freq)
    | [] -> raise Not_found
    | _ -> failwith "Error: you have more than one 'autopilot' section in your airframe file"

(** [get_targets_of_module xml]
    * Returns the list of targets of a module *)
let get_targets_of_module = fun xml ->
  let targets = Xml.map
      (fun x ->
	match String.lowercase (Xml.tag x) with
	| "makefile" -> targets_of_field x Env.default_module_targets
	| _ -> []
      ) xml in
  singletonize (List.flatten targets)

let module_name = fun xml ->
  let name = ExtXml.attrib xml "name" in
  try Filename.chop_extension name with _ -> name

let get_module = fun m global_targets ->
  match Xml.tag m with
  | "module" ->
      let name = module_name m in
      let filename =
	let modtype = ExtXml.attrib_or_default m "type" "" in
	name ^ (if modtype = "" then "" else "-") ^ modtype ^ ".xml" in
      let file = modules_dir // filename in
      let xml = ExtXml.parse_file file in
      let targets = get_targets_of_module xml in
      let targets = union global_targets targets in
      { name = name; xml = xml; file = file; filename = filename; vpath = None;
	param = Xml.children m; targets = targets }
  | "load" -> (* this case should be removed after transition phase *)
      let dir, vpath =
	try
	  let dir = ExtXml.attrib m "dir" in
	  let dir =
	    if Filename.is_relative dir then Env.paparazzi_home // dir
	    else dir in
	  (dir, Some dir)
        with _ -> modules_dir, None in
      let filename = ExtXml.attrib m "name" in
      let name = Filename.chop_extension filename in
      let file = dir // filename in
      let xml = ExtXml.parse_file file in
      let targets = get_targets_of_module xml in
      let extra_targets = global_targets @ targets_of_field m "" in
      let targets = singletonize (extra_targets @ targets) in
      { name = name; xml = xml; file = file; filename = filename; vpath = vpath;
	param = Xml.children m; targets = targets }
  | _ -> Xml2h.xml_error "module or load"

(** [get_modules_of_airframe xml]
    * Returns a list of module configuration from airframe file *)
let rec get_modules_of_airframe = fun xml ->
  let is_module = fun tag -> List.mem tag [ "module"; "load" ] in
  let rec iter_modules = fun targets modules xml ->
    match xml with
    | Xml.PCData _ -> modules
    | Xml.Element (tag, _attrs, children) when is_module tag ->
	let m = get_module xml targets in
	List.fold_left
	  (fun acc xml -> iter_modules targets acc xml) (m :: modules) children
    | Xml.Element (tag, _attrs, children) ->
	let targets =
	  if tag = "modules" then targets_of_field xml "" else targets in
	List.fold_left
	  (fun acc xml -> iter_modules targets acc xml) modules children in
  let modules = iter_modules [] [] xml in
  let ap_modules =
    try
      let ap_file = fst (get_autopilot_of_airframe xml) in
      iter_modules [] [] (ExtXml.parse_file ap_file)
    with _ -> [] in
  ap_modules @ modules

(** [unload_unused_modules modules ?print_error]
    * Returns a list of [modules] where unused modules are removed
    * If [print_error] is true, a warning is printed *)
let unload_unused_modules = fun modules print_error ->
  let target = try Sys.getenv "TARGET" with _ -> "" in
  if String.length target = 0 then modules
  else List.filter (fun m -> List.mem target m.targets) modules

(** [get_modules_name xml]
    * Returns a list of loaded modules' name *)
let get_modules_name = fun xml ->
  (* extract all "modules" sections *)
  let modules = get_modules_of_airframe xml in
  (* filter the list if target is not supported *)
  let modules = unload_unused_modules modules false in
  (* return a list of modules name *)
  List.map (fun m -> ExtXml.attrib m.xml "name") modules

(** [get_modules_dir xml]
    * Returns the list of modules directories *)
let get_modules_dir = fun modules ->
  let dir = List.map (fun m -> try Xml.attrib m.xml "dir" with _ -> ExtXml.attrib m.xml "name") modules in
  singletonize (List.sort compare dir)
