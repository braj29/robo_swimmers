from __future__ import absolute_import

from ..spec.exception import err
from ..spec.msgs import Body
from ..spec.msgs import BodyPart
from ..spec.msgs import NeuralNetwork


class BodyDecoder(object):
    """
    Body decoder for the default YAML object structure
    """

    def __init__(self, spec):
        """
        :param spec:
        :type spec: BodyImplementation
        :return:
        """
        self.spec = spec
        self.part_ids = set()

    def decode(self, obj):
        """
        :param obj:
        :return:
        """
        if 'body' not in obj:
            err("Missing robot body.")

        body = Body()
        body.root.CopyFrom(self._process_body_part(obj['body']))
        return body

    def _process_body_part(
            self,
            part,
            dst_slot=None,
            x=0,
            y=0,
            orientation=0):
        """
        :param part:
        :return:
        :rtype: BodyPart
        """
        proto_part = BodyPart()
        proto_part.x = x
        proto_part.y = y

        if 'id' not in part:
            err("Missing part ID.")

        proto_part.id = part_id = part['id']
        if part_id in self.part_ids:
            err("Duplicate part ID '{}'".format(part_id))
        self.part_ids.add(part_id)

        if 'type' not in part:
            err("Missing part type.")
        proto_part.type = part_type = part['type']

        proto_template = self.spec.get(part_type)
        if proto_template is None:
            err("Part type '{}' not in implementation spec.".format(part_type))

        # Check destination slot arity
        if dst_slot is not None and dst_slot >= proto_template.arity:
            err("Cannot attach part '%s' with arity %d at slot %d" %
                (part_id, proto_template.arity, dst_slot))

        # Add part parameters
        proto_part.orientation = part.get('orientation', 0)

        params = proto_template.serialize_params(part.get('params', {}))
        for param in params:
            p = proto_part.param.add()
            p.value = param

        # Add children
        children = part.get('children', {})
        for child_id in children:
            if child_id >= proto_template.arity:
                err("Cannot attach to slot {} of part '{}' with arity "
                    "{}.".format(child_id, part_id, proto_template.arity))

            if child_id == dst_slot:
                err("Part '{}': Attempt to use slot {} for child which is "
                    "already attached to parent.".format(part_id, child_id))
            childs_x, childs_y = self._coordinates(
                    orientation=orientation,
                    child_slot=child_id,
                    init_x=proto_part.x,
                    init_y=proto_part.y)
            self._process_body_connection(
                    parent_part=proto_part,
                    parent_slot=child_id,
                    child_part=children[child_id],
                    x=childs_x,
                    y=childs_y,
                    orientation=orientation)

        return proto_part

    def _process_body_connection(
            self,
            parent_part,
            parent_slot,
            child_part,
            x,
            y,
            orientation
    ):
        """
        :param parent_part:
        :type parent_part: BodyPart
        :param parent_slot: Slot on parent
        :type parent_slot: int
        :param child_part:
        :return:
        :rtype: BodyConnection
        """
        conn = parent_part.child.add()
        conn.src_slot = parent_slot
        conn.dst_slot = child_part['slot'] if 'slot' in child_part else 0
        childs_orientation = self._orientation(
                parents_slot=parent_slot,
                parents_orientation=orientation)
        conn.part.CopyFrom(self._process_body_part(
                part=child_part,
                dst_slot=conn.dst_slot,
                x=x,
                y=y,
                orientation=childs_orientation))

    @staticmethod
    def _orientation(parents_slot, parents_orientation):
        """
        Method that determines the rotation of a module
        :param arity:
        :param parents_slot:
        :param parents_orientation:
        :return:
        :rtype: int
        """
        orientation = int(parents_orientation)
        if orientation == 0:
            if parents_slot == 0:
                return 2
            elif parents_slot == 1:
                return 0
            elif parents_slot == 2:
                return 3
            elif parents_slot == 3:
                return 1
        elif orientation == 1:
            if parents_slot == 0:
                return 3
            elif parents_slot == 1:
                return 1
            elif parents_slot == 2:
                return 0
            elif parents_slot == 3:
                return 2
        elif orientation == 2:
            if parents_slot == 0:
                return 0
            elif parents_slot == 1:
                return 2
            elif parents_slot == 2:
                return 1
            elif parents_slot == 3:
                return 3
        elif orientation == 3:
            if parents_slot == 0:
                return 1
            elif parents_slot == 1:
                return 3
            elif parents_slot == 2:
                return 2
            elif parents_slot == 3:
                return 0
        else:
            err("Unsupported parents rotation angle provided.")

    @staticmethod
    def _coordinates(orientation, child_slot, init_x, init_y):
        """
        Method that determines the coordinates of a module
        :param orientation:
        :param init_x:
        :param init_y:
        :return:
        :rtype: tuple
        """
        orientation = int(orientation)
        if orientation == 0:
            if child_slot == 0:
                return init_x, (init_y - 1)
            elif child_slot == 1:
                return init_x, (init_y + 1)
            elif child_slot == 2:
                return (init_x + 1), init_y
            elif child_slot == 3:
                return (init_x - 1), init_y
        elif orientation == 1:
            if child_slot == 0:
                return (init_x + 1), init_y
            elif child_slot == 1:
                return (init_x - 1), init_y
            elif child_slot == 2:
                return init_x, (init_y + 1)
            elif child_slot == 3:
                return init_x, (init_y - 1)
        elif orientation == 2:
            if child_slot == 0:
                return init_x, (init_y + 1)
            elif child_slot == 1:
                return init_x, (init_y - 1)
            elif child_slot == 2:
                return (init_x - 1), init_y
            elif child_slot == 3:
                return (init_x + 1), init_y
        elif orientation == 3:
            if child_slot == 0:
                return (init_x - 1), init_y
            elif child_slot == 1:
                return (init_x + 1), init_y
            elif child_slot == 2:
                return init_x, (init_y - 1)
            elif child_slot == 3:
                return init_x, (init_y + 1)
        else:
            err("Unsupported parents rotation angle provided.")


class NeuralNetworkDecoder(object):
    """
    Decoder class for the standard neural network spec.
    """

    def __init__(self, spec, body_spec):
        """
        :param spec:
        :type spec: NeuralNetImplementation
        :param body_spec:
        :type body_spec: BodyImplementation
        :return:
        """
        self.spec = spec
        self.body_spec = body_spec
        self.neurons = {}

    def decode(self, obj):
        """
        :param obj:
        :return:
        :rtype: NeuralNetwork
        """
        if 'body' not in obj:
            err("Robot body required for standard Neural Network decode.")

        # Prepare all automatic input / output neurons
        self._process_body_part(obj['body'])

        brain = obj.get('brain', {})
        neurons = brain.get('neurons', [])
        params = brain.get('params', {})
        connections = brain.get('connections', [])

        self._create_hidden_neurons(neurons)

        # Process given parameters
        for neuron_id in params:
            self._process_neuron_params(neuron_id, params[neuron_id])

        nn = NeuralNetwork()
        self._process_neurons(nn)
        self._create_neuron_connections(connections, nn)

        return nn

    def _process_body_part(self, conf):
        """
        :param conf:
        :return:
        :rtype: BodyPart
        """
        part_id = conf['id']
        part_type = conf['type']

        spec = self.body_spec.get(part_type)
        if spec is None:
            err("Part type '{}' not in implementation spec.".format(part_type))

        # Add children
        children = conf.get('children', {})
        for src in children:
            self._process_body_part(children[src])

        # Add automatic input / output neurons
        cats = {"in": spec.inputs, "out": spec.outputs}
        for cat in cats:
            default_type = "Input" if cat == "in" else "Simple"

            for i in range(cats[cat]):
                neuron_id = "{}-{}-{}".format(part_id, cat, i)
                if neuron_id in self.neurons:
                    err("Duplicate neuron ID '{}'".format(neuron_id))

                self.neurons[neuron_id] = {
                    "layer": "{}put".format(cat),
                    "part_id": part_id
                }

                self._process_neuron_params(neuron_id, {"type": default_type})

    def _process_neuron_params(self, neuron_id, conf):
        """
        Processes params for a single neuron.
        :param neuron_id:
        :param conf:
        :return:
        """
        if neuron_id not in self.neurons:
            err("Cannot set parameters for unknown neuron '{}'".format(
                    neuron_id))

        current = self.neurons[neuron_id]
        if "type" not in current or "type" in conf:
            current["type"] = conf.get("type", "Simple")

        if current["type"] != "Input" and current["layer"] == "input":
            err("Input neuron '{}' must be of type 'Input'".format(neuron_id))

        spec = self.spec.get(current["type"])
        if spec is None:
            err("Unknown neuron type '{}'".format(current["type"]))

        current["params"] = spec.serialize_params(conf)

    def _create_hidden_neurons(self, neurons):
        """
        Creates hidden neurons.
        :return:
        """
        for n_id in neurons:
            if n_id in self.neurons:
                err("Duplicate neuron ID '{}'".format(n_id))

            # This sets the defaults, the accurate values - if present - will
            # be set by `_process_neuron_params`.
            self.neurons[n_id] = {
                "layer": "hidden",
                "type": "Simple"
            }

            if "part_id" in neurons[n_id]:
                self.neurons[n_id]["part_id"] = neurons[n_id]["part_id"]

            self._process_neuron_params(n_id, neurons[n_id])

    def _create_neuron_connections(self, connections, brain):
        """
        Creates connections from the robot connection list.
        :param connections:
        :param brain:
        :return:
        """
        for conn in connections:
            c = brain.connection.add()
            src = conn.get("src", None)
            dst = conn.get("dst", None)
            c.weight = conn.get("weight", 0)

            if src is None:
                err("Neuron connection is missing 'src'.")

            if src not in self.neurons:
                err("Using unknown neuron '{}' as connection source.".format(
                        src))

            if dst is None:
                err("Neuron connection is missing 'dst'.")

            if dst not in self.neurons:
                err("Using unknown neuron '{}' as connection "
                    "destination.".format(dst))

            if self.neurons[dst]["layer"] == "input":
                err("Using input neuron '{}' as destination.".format(dst))

            c.src = src
            c.dst = dst

    def _process_neurons(self, brain):
        """
        Processes neuron data into protobuf neurons.
        :param brain:
        :type brain: NeuralNetwork
        :return:
        """
        for neuron_id in self.neurons:
            conf = self.neurons[neuron_id]
            neuron = brain.neuron.add()
            neuron.id = neuron_id
            neuron.layer = conf["layer"]
            neuron.type = conf["type"]

            if "part_id" in conf:
                neuron.partId = conf["part_id"]

            for value in conf["params"]:
                param = neuron.param.add()
                param.value = value
