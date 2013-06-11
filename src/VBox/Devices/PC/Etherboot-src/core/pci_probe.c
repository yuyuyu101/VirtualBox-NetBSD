#ifdef CONFIG_PCI
/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2, or (at
 * your option) any later version.
 */

/*
 * Oracle GPL Disclaimer: For the avoidance of doubt, except that if any license choice
 * other than GPL or LGPL is available it will apply instead, Oracle elects to use only
 * the General Public License version 2 (GPLv2) at this time for any software where
 * a choice of GPL license versions is made available with the language indicating
 * that GPLv2 or any later version may be used, or where a choice of which version
 * of the GPL is applied is otherwise unspecified.
 */

#include	"etherboot.h"
#include	"nic.h"
#include	"pci.h"

void pci_enumerate(void)
{
	const struct pci_driver *driver;
	for(driver = pci_drivers; driver < pci_drivers_end; driver++) {
		printf("%s ", driver->name);
	}
}

int pci_probe(struct dev *dev, const char *type_name)
{
/*
 *	NIC probing is in pci device order, followed by the 
 *      link order of the drivers.  A driver that matches 
 *      on vendor and device id will supersede a driver
 *      that matches on pci class.
 *
 *	If you want to probe for another device behind the same pci
 *      device just increment index.  And the previous probe call
 *      will be repeated.
 */
	struct pci_probe_state *state = &dev->state.pci;
#ifndef VBOX
	printf("Probing pci %s...\n", type_name);
#endif /* !VBOX */
	if (dev->how_probe == PROBE_FIRST) {
		state->advance    = 1;
		state->dev.driver = 0;
		state->dev.bus    = 0;
		state->dev.devfn  = 0;
		dev->index        = -1;
	}
	for(;;) {
		if ((dev->how_probe != PROBE_AWAKE) && state->advance) {
			find_pci(dev->type, &state->dev);
			dev->index = -1;
		}
		state->advance = 1;
		
		if (state->dev.driver == 0)
			break;
		
		if (dev->how_probe != PROBE_AWAKE) {
			dev->type_index++;
		}
		dev->devid.bus_type = PCI_BUS_TYPE;
		dev->devid.vendor_id = htons(state->dev.vendor);
		dev->devid.device_id = htons(state->dev.dev_id);
		/* FIXME how do I handle dev->index + PROBE_AGAIN?? */
		
#ifndef VBOX
		printf("[%s]", state->dev.name);
#endif /* !VBOX */
		if (state->dev.driver->probe(dev, &state->dev)) {
			state->advance = (dev->index == -1);
			return PROBE_WORKED;
		}
		putchar('\n');
	}
	return PROBE_FAILED;
}

#endif
